import reconos.utils.shutil2 as shutil2
import reconos.utils.template as template

import logging
import argparse

import tempfile
import subprocess

import os
from os import listdir
from os.path import isfile, join, isdir

log = logging.getLogger(__name__)

def get_cmd(prj):
	return "export_hw"

def get_call(prj):
	return export_hw_cmd

def get_parser(prj):
	parser = argparse.ArgumentParser("export_hw", description="""
		Exports the hardware project and generates all necessary files.
		""")
	parser.add_argument("-l", "--link", help="link sources instead of copying", action="store_true")
	parser.add_argument("-t", "--thread", help="export only single thread")
	parser.add_argument("hwdir", help="alternative export directory", nargs="?")
	return parser

def get_dict(prj):
	dictionary = {}
	dictionary["NUM_SLOTS"] = len(prj.slots)
	dictionary["NUM_CLOCKS"] = len(prj.clocks)
	dictionary["SYSCLK"] = prj.clock.id
	dictionary["SYSRST"] = "SYSRESET"
	dictionary["TOOL"]   = prj.impinfo.xil[0]
	dictionary["SLOTS"] = []
	for s in prj.slots:
		if s.threads:
			d = {}
			d["_e"] = s
			d["HwtCoreName"] = s.threads[0].get_corename()
			d["HwtCoreVersion"] = s.threads[0].get_coreversion()
			d["Id"] = s.id
			d["SlotId"] = s.id
			d["Name"] = s.name.lower()
			d["Clk"] = s.clock.id
			d["Async"] = "sync" if s.clock == prj.clock else "async"
			d["Ports"] = s.ports
			d["Reconfigurable"] = prj.impinfo.pr
			d["Region"] = []
			for reg in s.region:
				e = {}
				e["RegionArea"] = reg 
				d["Region"].append(e)

			# Dict for nested generate statement
			d["THREADS"] = []
			for t in s.threads:
				d2 = {}
				d2["Name"] = t.name.lower()
				d["THREADS"].append(d2)
			dictionary["SLOTS"].append(d)
	# Prepare strings to define RM configurations because it is difficult to handle purely with template functionality
	dictionary["THREADS"] = []
	config_id = 0
	for t in prj.threads:
		d = {}
		if config_id == 0:
			rm_configuration = "set rm_config(initial) \""
		else:
			rm_configuration = "set rm_config(reconfig_" + str(config_id) + ") \""

		# Configuration uses default thread in slots which are not associated with the current thread, since we need to specify a module for every partition
		for s in prj.slots:
			if s in t.slots:
				rm_configuration += " $rp" + str(s.id) + " $rp" + str(s.id) + "_inst " + t.name.lower() + "_" + str(s.id)
			else:
				rm_configuration += " $rp" + str(s.id) + " $rp" + str(s.id) + "_inst " + "reconf" + "_" + str(s.id)
		rm_configuration += "\""
		d["RMConfiguration"] = rm_configuration
		dictionary["THREADS"].append(d)
		config_id += 1

	dictionary["CLOCKS"] = []
	for c in prj.clocks:
		d = {}
		d["Id"] = c.id
		d["ReqFreqKHz"] = c.freq / 1000
		param = c.get_pllparam(800000000, 1600000000, 100000000)
		d["ActFreqKHz"] = param[2] / 1000
		d["M"] = param[0]
		d["O"] = param[1]
		dictionary["CLOCKS"].append(d)
	
	return dictionary
	
def export_hw_cmd(args):
	if args.thread is None:
		export_hw(args.prj, args.hwdir, args.link)
	else:
		export_hw_thread(args.prj, args.hwdir, args.link, args.thread)

def export_hw(prj, hwdir, link):
	if prj.impinfo.xil[0] == "vivado":
		_export_hw_vivado(prj, hwdir, link)
	else:
		log.error("Tool not supported")

def export_hw_thread(prj, hwdir, link, thread):
	if prj.impinfo.xil[0] == "vivado":
		_export_hw_thread_vivado(prj, hwdir, link, thread)
	else:
		log.error("Tool not supported")

def _export_hw_thread_vivado(prj, hwdir, link, thread):
	''' 
	Generates sources for one hardware thread for ReconOS in a Vivado project.
	
	It checks whether vhdl or hls sources shall be used and generates the hardware thread
	from the source templates. 
	
	hwdir gives the name of the project directory
	link boolean; if true files will be linked instead of copied
	thread is the name of the hardware thread to generate
	'''
	hwdir = hwdir if hwdir is not None else prj.basedir + ".hw" + "." + thread.lower()

	log.info("Exporting thread " + thread + " to directory '" + hwdir + "'")

	threads = [_ for _ in prj.threads if _.name == thread]
	if (len(threads) == 1):
		thread = threads[0]

		if thread.hwsource is None:
			log.info("No hardware source specified")
	else:
		log.error("Thread '" + thread  + "' not found")
		return

	if thread.hwsource == "vhdl":
		dictionary = {}
		dictionary["ID"] = thread.id
		dictionary["NAME"] = thread.name.lower()
		dictionary["MEM"] = thread.mem
		dictionary["MEM_N"] = not thread.mem
		dictionary["CLKPRD"] = min([_.clock.get_periodns() for _ in thread.slots])
		dictionary["HWSOURCE"] = thread.hwsource
		# "reconf" thread for partial reconfiguration is taken from template directory
		if prj.impinfo.pr and thread.name.lower() == "reconf":
			if prj.impinfo.cpuarchitecture == "arm64":
				srcs = shutil2.join(prj.get_template("thread_rt_reconf64"), thread.hwsource)
			else:
				srcs = shutil2.join(prj.get_template("thread_rt_reconf"), thread.hwsource)
		else:
			srcs = shutil2.join(prj.dir, "src", "rt_" + thread.name.lower(), thread.hwsource)
		dictionary["SOURCES"] = [srcs]
		incls = shutil2.listfiles(srcs, True)
		dictionary["INCLUDES"] = [{"File": shutil2.trimext(_)} for _ in incls]
		dictionary["RESOURCES"] = []
		for i, r in enumerate(thread.resources):
			d = {}
			d["NameUpper"] = (r.group + "_" + r.name).upper()
			d["NameLower"] = (r.group + "_" + r.name).lower()
			d["LocalId"] = i
			d["HexLocalId"] =  "%08x" % i
			dictionary["RESOURCES"].append(d)
		dictionary["PORTS"] = thread.ports

		log.info("Generating export files ...")
		prj.apply_template("thread_vhdl_pcore", dictionary, hwdir, link)

		#For each slot: Generate .prj file listing sources for PR flow
		if prj.impinfo.pr:
			for _ in thread.slots:
				dictionary["SLOTID"] = _.id
				prj.apply_template("thread_prj", dictionary, hwdir, link)

	elif thread.hwsource == "hls":
		tmp = tempfile.TemporaryDirectory()

		dictionary = {}

		#Added
		dictionary["MSGINCLUDEDIR"] = ""
		msg_install_path = prj.dir + "/build.msg/install/"
		if shutil2.exists(msg_install_path):
			msg_packages = [f for f in listdir(msg_install_path) if isdir(join(msg_install_path, f))]
			#print(msg_packages)
			for msg_pack in msg_packages:
				dictionary["MSGINCLUDEDIR"] += "-I"+msg_install_path + msg_pack + "/include/ "
		#End Added

		#if shutil2.exists(prj.dir + "/hls_include/Vitis_Libraries/vision/L1/include"):
		#	dictionary["MSGINCLUDEDIR"] += "-I"+prj.dir + "/hls_include/Vitis_Libraries/vision/L1/include "

		dictionary["MSGINCLUDEDIR"] += prj.impinfo.hls_cflags

		dictionary["PART"] = prj.impinfo.part
		dictionary["NAME"] = thread.name.lower()
		dictionary["MEM"] = thread.mem
		dictionary["MEM_N"] = not thread.mem
		dictionary["CLKPRD"] = min([_.clock.get_periodns() for _ in thread.slots])
		
		if prj.impinfo.cpuarchitecture == "arm64":
			dictionary["CPUARCHITECTURESIZE"] = "64"
		else:
			dictionary["CPUARCHITECTURESIZE"] = "32"

		if prj.impinfo.cpuarchitecture == "arm64":
			dictionary["RRBASETYPE"] 		= "int64_t"
			dictionary["RRUBASETYPE"] 		= "uint64_t"
			dictionary["RRBASETYPEBYTES"] 	= "8"
		else:
			dictionary["RRBASETYPE"] 		= "int32_t"
			dictionary["RRUBASETYPE"] 		= "uint32_t"
			dictionary["RRBASETYPEBYTES"] 	= "4"


		dictionary["ROSDISTRIBUTION"] = prj.impinfo.ros2distribution

		reconf_of_slots_or = False
		reconf_of_slots_and = True

		for e in thread.slots:
			reconf_of_slots_and = reconf_of_slots_and and e.reconfigurable
			reconf_of_slots_or = reconf_of_slots_or or e.reconfigurable

		if reconf_of_slots_or != reconf_of_slots_and:
			log.error("Slots of thread {} have to be either reconfigurable or not".format_map(thread.name))
			return

		
		# "reconf" thread for partial reconfiguration is taken from template directory
		if reconf_of_slots_or and thread.name.lower() == "reconf":
			srcs = shutil2.join(prj.get_template("thread_rt_reconf"), thread.hwsource)
		else:
			srcs = shutil2.join(prj.dir, "src", "rt_" + thread.name.lower(), thread.hwsource)
		dictionary["SOURCES"] = [srcs]
		files = shutil2.listfiles(srcs, True)
		dictionary["FILES"] = [{"File": _} for _ in files]
		dictionary["RESOURCES"] = []
		for i, r in enumerate(thread.resources):
			d = {}
			d["NameUpper"] = (r.group + "_" + r.name).upper()
			d["NameLower"] = (r.group + "_" + r.name).lower()
			d["LocalId"] = i
			d["HexLocalId"] =  "%08x" % i
			d["Type"] = r.type
			d["TypeUpper"] = r.type.upper()
			dictionary["RESOURCES"].append(d)

		log.info("Generating temporary HLS project in " + tmp.name + " ...")
		prj.apply_template("thread_hls_build", dictionary, tmp.name)

		log.info("Starting Vivado HLS ...")

		subprocess.call("""
			source /opt/Xilinx/Vivado/{1}/settings64.sh;
			cd {0};
			vivado_hls -f script_csynth.tcl;
			vivado -mode batch -notrace -nojournal -nolog -source script_vivado_edn.tcl;""".format(tmp.name, prj.impinfo.hls[1]),
			shell=True, executable="/bin/bash")


		dictionary = {}
		dictionary["NAME"] = thread.name.lower()
		dictionary["MEM"] = thread.mem
		dictionary["MEM_N"] = not thread.mem
		dictionary["HWSOURCE"] = thread.hwsource
		srcs = shutil2.join(tmp.name, "hls", "sol", "syn", "vhdl")
		#HLS instantiates subcores (e.g. floating point units) in VHDL form during the export step
		#The path above contains only .tcl instantiations, which our IP Packager flow doesn't understand
		#So we add extract the convenient .vhd definitions from the following path:
		srcs2 = shutil2.join(tmp.name, "hls", "sol", "impl", "ip", "hdl", "ip")
		dictionary["SOURCES"] = [srcs, srcs2]
		incls = shutil2.listfiles(srcs, True)
		incls += shutil2.listfiles(srcs2, True)
		dictionary["INCLUDES"] = [{"File": shutil2.trimext(_)} for _ in incls]
		#Template will change top module entity name to "rt_reconf" if PR flow is used for this HWT
		dictionary["RECONFIGURABLE"] = reconf_of_slots_or#  prj.impinfo.pr

		log.info("Generating export files ...")
		if prj.impinfo.cpuarchitecture == "arm64":
			prj.apply_template("thread_hls_pcore_vhdl_64", dictionary, hwdir)
		else:
			prj.apply_template("thread_hls_pcore_vhdl", dictionary, hwdir)
		

		#For each slot: Generate .prj file listing sources for PR flow
		if prj.impinfo.pr:
			for _ in thread.slots:
				dictionary["SLOTID"] = _.id
				prj.apply_template("thread_prj", dictionary, hwdir, link)

		#Save temporary HLS project directory for analysis:
		shutil2.mkdir("/tmp/ReconROS_hlsexport")
		save_dir_hls_prj = shutil2.join(hwdir, "..", "tmp_hls_prj_" + thread.name.lower())
		shutil2.copytree(tmp.name, "/tmp/ReconROS_hlsexport")
		shutil2.rmtree(save_dir_hls_prj)
		shutil2.mkdir(save_dir_hls_prj)
		shutil2.copytree(tmp.name, save_dir_hls_prj)
	else:
		log.error("No source type defined")
		return
		
def _export_hw_vivado(prj, hwdir, link):
	''' 
	Generates a TCL script for generation of a Vivado based project.
	
	It first compiles the configuration dictionary and then processes the templates
	according to the configuration dictionary.
	
	hwdir gives the name of the project directory
	link boolean; if true files will be linked instead of copied
	'''
	
	print("export_hw_vivado")
	hwdir = hwdir if hwdir is not None else prj.basedir + ".hw"

	log.info("Export hardware to directory '" + hwdir + "'")

	dictionary = get_dict(prj)

	log.info("Generating export files ...")
	
	
	tmpl = "ref_" + prj.impinfo.os + "_" + "_".join(prj.impinfo.board) + "_" + prj.impinfo.design + "_" + prj.impinfo.xil[0] + "_" + prj.impinfo.xil[1]
	print("Using template directory " + tmpl)
	if not shutil2.exists(prj.get_template(tmpl)):
		log.error("Template directory not found in project or ReconOS repository")
		return
	prj.apply_template(tmpl, dictionary, hwdir, link)

	log.info("Generating threads ...")
	for t in prj.threads:
		export_hw_thread(prj, shutil2.join(hwdir, "pcores"), link, t.name)
		
	print("Calling TCL script to generate Vivado IP Repository")
	result = subprocess.call("""
					source /opt/Xilinx/Vivado/{1}/settings64.sh;
					cd {0};
					vivado -mode batch -notrace -nojournal -nolog -source create_ip_library.tcl;""".format(hwdir, prj.impinfo.xil[1]),
					shell=True,executable="/bin/bash")
	if result != 0 :
		print("[RDK] Generation of Vivado IP repository failed. Maybe you specified unknown components in build.cfg?")
		exit(1)
	
	print("Calling TCL script to generate ReconOS in Vivado IP Integrator")
	subprocess.call("""
					source /opt/Xilinx/Vivado/{1}/settings64.sh;
					cd {0};
					vivado -mode batch -notrace -nojournal -nolog -source export.tcl -tclargs -proj_name myReconOS -proj_path . ;""".format(hwdir, prj.impinfo.xil[1]),
					shell=True,  executable="/bin/bash")

