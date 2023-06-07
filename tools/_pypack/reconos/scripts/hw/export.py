import reconos.utils.custom_msgs as custom_msgs
import reconos.utils.shutil2 as shutil2
import reconos.utils.template as template
import reconos.utils.msg_parsing as mp

import logging
import argparse

import tempfile
import subprocess

import time
import os
from os import listdir
from os.path import isfile, join, isdir
from pathlib import Path

from threading import Thread

log = logging.getLogger(__name__)


import shutil, errno

def copyanything(src, dst):
    try:
        shutil.copytree(src, dst)
    except OSError as exc: # python >2.5
        if exc.errno in (errno.ENOTDIR, errno.EINVAL):
            shutil.copy(src, dst)
        else: raise


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
			d["Reconfigurable"] = s.reconfigurable
			d["Region"] = []
			for reg in s.region:
				e = {}
				e["RegionArea"] = reg 
				log.debug(reg)
				d["Region"].append(e)

			# Dict for nested generate statement
			d["THREADS"] = []
			for t in s.threads:
				d2 = {}
				d2["Name"] = t.name.lower()
				d["THREADS"].append(d2)
			dictionary["SLOTS"].append(d)


	dictionary["HWTOPICS"] = []

	


	for i, topic in enumerate([_ for _ in prj.resources if (_.group == "__global_ressource_group___") and (_.type == "hwtopic")]):
		d = {}
		log.debug("-----topic------")
		log.debug(topic)
		topic.name = topic.name.replace("/","")
		d["Name"] = topic.name
		d["MsgType"] = topic.args[1]
		d["SUBSCRIBERS"] = []
		d["PUBLISHERS"] = []
		d["ROSGATEWAYS"] = []
		d["Type"] = ""
		num_hw_subs = 0
		num_hw_pubs = 0

		for s in prj.slots:
			if s.threads:
				log.debug(s.threads)
				for t in s.threads:
					log.debug("thread resources:")
					log.debug(str(t.resources))
					log.debug("-------")	
				
				for i, sub in enumerate([_ for _ in t.resources if (_.type == "hwtopicsub") and (_.name.replace("/","")==topic.name)]):
					dd = {}

					log.debug("found subscriber {}".format(t.name))
					if len(sub.args) == 2:
						dd["FIFOSIZE"] = sub.args[1]
						dd["FIFOID"]   = sub.id
					else:
						dd["FIFOSIZE"] =  "0"

					log.debug("FIFOSIZE = " + dd["FIFOSIZE"])

					dd["SlotId"] =  s.id
					if hasattr(s.threads[0], 'hwtopic'):
						dd["SlotPortName"] = topic.name + "_in"
					else:
						dd["SlotPortName"] = topic.name
					dd["SubNr"] = num_hw_subs
					d["SUBSCRIBERS"].append(dd)
					num_hw_subs = num_hw_subs + 1
				
				for i, pub in enumerate([_ for _ in t.resources if (_.type == "hwtopicpub") and (_.name.replace("/","")==topic.name)]):
					log.debug(pub,i)
					dd = {}
					log.debug("found publisher {}".format(t.name))
					dd["SlotId"] =  s.id
					if hasattr(s.threads[0], 'hwtopic'):
						dd["SlotPortName"] = topic.name + "_out"
					else:
						dd["SlotPortName"] = topic.name
					dd["PubNr"] = num_hw_pubs
					d["PUBLISHERS"].append(dd)
					num_hw_pubs = num_hw_pubs + 1

			#else:
			#	print("ROSGateway: Different topic names ("+rosgw.hwtopic.replace("/","")+","+topic.name+")")
			

		d["NUM_SUBS"] = num_hw_subs
		d["NUM_PUBS"] = num_hw_pubs

		if((num_hw_pubs == 1) and (num_hw_subs == 1)):
			d["Type"] = "ONE_TO_ONE"
			log.debug("Found 1-to-1 config. Directly connecting pub/sub for hardware topic ", topic.name)
			d["SUBSCRIBERS"][0]["Pub_SlotId"] = d["PUBLISHERS"][0]["SlotId"]
			d["SUBSCRIBERS"][0]["Pub_SlotPortName"] = d["PUBLISHERS"][0]["SlotPortName"]

		if((num_hw_pubs == 1) and (num_hw_subs > 1)):
			d["Type"]  = "ONE_TO_N"
			log.debug("Found 1-to-N config. Selecting AXIS-Broadcaster-based architecture for hardware topic ", topic.name)

		if((num_hw_pubs > 1) and (num_hw_subs == 1)):
			d["Type"]  = "N_TO_ONE"
			log.debug("Found N-to-1 config. Selecting AXIS-Interconnect-based architecture for hardware topic ", topic.name)

		if((num_hw_pubs > 1) and (num_hw_subs > 1)):
			d["Type"]  = "N_TO_M"
			log.debug("Found N-to-M config. Selecting AXIS-Interconnect and Broadcaster-based architecture for hardware topic ", topic.name)

		if(num_hw_subs <= 1):
			d["NUM_AXIS_BR_SUBS"] = 2
		else:
			d["NUM_AXIS_BR_SUBS"] = num_hw_subs
		dictionary["HWTOPICS"].append(d)


	dictionary["THREADS"] = []
	config_id = 0

	for t in [_ for _ in prj.threads if _.has_reconfig_slots == True]:
		
		d = {}
		if config_id == 0:
			rm_configuration = "set rm_config(initial) \""
		else:
			rm_configuration = "set rm_config(reconfig_" + str(config_id) + ") \""

		# Configuration uses default thread in slots which are not associated with the current thread, since we need to specify a module for every partition
		for s in [_ for _ in prj.slots if _.reconfigurable == True]:
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

def export_hw_rosgateway(prj, hwdir, link, thread):
	if prj.impinfo.xil[0] == "vivado":
		_export_hw_rosgateway_vivado(prj, hwdir, link, thread)
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
		#dictionary["INCLUDES"] = [{"FilewithExtension": shutil2.trimext(_) + shutil2.getext(_)} for _ in [item for item in incls if shutil2.getext(item) != ".v" and shutil2.getext(item) != ".tcl"]]
		dictionary["INCLUDES"] = [{"FilewithExtension": shutil2.trimext(_) + shutil2.getext(_)} for _ in [item for item in incls if shutil2.getext(item) != ".tcl"]]
		dictionary["RESOURCES"] = []
		for i, r in enumerate(thread.resources):
			d = {}
			d["NameUpper"] = (r.group + "_" + r.name).upper()
			d["NameLower"] = (r.group + "_" + r.name).lower()
			d["LocalId"] = i
			d["HexLocalId"] =  "%08x" % i
			dictionary["RESOURCES"].append(d)
		dictionary["PORTS"] = thread.ports

		if prj.impinfo.hls[1] == "2021.2":
			dictionary["VIVADO"] = "2021"
		elif prj.impinfo.hls[1] == "2022.1":
			dictionary["VIVADO"] = "2022"	
		else:
			dictionary["VIVADO"] = "2020"	

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
		for msg_include_dir in custom_msgs.get_absolute_include_paths(prj.dir):
			dictionary["MSGINCLUDEDIR"] += "-I" + msg_include_dir + " "
		#End Added

		#if shutil2.exists(prj.dir + "/hls_include/Vitis_Libraries/vision/L1/include"):
		#	dictionary["MSGINCLUDEDIR"] += "-I"+prj.dir + "/hls_include/Vitis_Libraries/vision/L1/include "

		dictionary["MSGINCLUDEDIR"] += prj.impinfo.hls_cflags

		dictionary["PART"] = prj.impinfo.part
		dictionary["NAME"] = thread.name.lower()
		dictionary["MEM"] = thread.mem
		dictionary["MEM_N"] = not thread.mem
		dictionary["CLKPRD"] = min([_.clock.get_periodns() for _ in thread.slots])

		if prj.impinfo.hls[1] == "2021.2":
			dictionary["VIVADO"] = "2021"
		elif prj.impinfo.hls[1] == "2022.1":
			dictionary["VIVADO"] = "2022"	
		else:
			dictionary["VIVADO"] = "2020"	
		
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
		for i, r in enumerate([_ for _ in thread.resources if (_.type != "hwtopicsub") and (_.type != "hwtopicpub")]):
			d = {}
			d["NameUpper"] = (r.group + "_" + r.name).upper()
			d["NameLower"] = (r.group + "_" + r.name).lower()
			d["LocalId"] = i
			d["HexLocalId"] =  "%08x" % i
			d["Type"] = r.type
			d["TypeUpper"] = r.type.upper()
			dictionary["RESOURCES"].append(d)


		
		dictionary["ROSMSG"] = []
		dictionary["HWPUBLISHERID"] = str(thread.id)

		# parsing ROS message definitions
		msglib_base_path = os.getenv('RECONOS') + "/lib/ros_msgs"
		paths = [msglib_base_path + "/common_interfaces", msglib_base_path + "/rcl_interfaces"]
		if shutil2.exists(prj.dir + "/msg"):
			paths += [prj.dir + "/msg"]
		msg_lib = mp.parse_msg_lib(paths)
		if(len(msg_lib) > 0):
			log.debug("-------------------------------Found msgs------------------------------")
			log.debug(msg_lib.keys())
		else:
			log.debug("Did not find any ROS-message definitions in the following paths:")
			for p in paths:
				log.debug(p)

		# build msg dictionary for subscriber of hwtopic
		dictionary["HWTOPICSSUB"] = []
		for r in [_ for _ in thread.resources if _.type == "hwtopicsub"]:
			d = {}
			d["HWPUBLISHERID"] = str(thread.id)
			d["Name"] = r.name.replace("/", "")
			d["NameUpper"] = r.name.replace("/", "").upper()
			my_msg = msg_lib[r.args[0]]
			temp = mp.build_msg_dict(my_msg, msg_lib, mp.primitive_lib)
			if(type(temp == dict)):
				log.debug("----- Sucessfully build msg dict for subscriber -----")
				log.debug("-----------------------------------------------------")
				log.debug(temp)
				d.update(temp)
				dictionary["ROSMSG"].append(temp)
			else:
				log.error("Could not build msg dict for subscriber of hwtopic")

			dictionary["HWTOPICSSUB"].append(d)
		# build msg dict for publisher of hwtopic
		dictionary["HWTOPICSPUB"] = []
		for r in [_ for _ in thread.resources if _.type == "hwtopicpub"]:
			d = {}
			d["Name"] = r.name.replace("/", "")
			d["NameUpper"] = r.name.replace("/", "").upper()
			my_msg = msg_lib[r.args[0]]
			temp = mp.build_msg_dict(my_msg, msg_lib, mp.primitive_lib)
			if(type(temp == dict)):
				log.debug("----- sucessfully build msg dict for publisher ------")
				log.debug("-----------------------------------------------------")
				log.debug(temp)
				d.update(temp)
				dictionary["ROSMSG"].append(temp)				
			else:
				log.error("Could not build msg dict for publisher of hwtopic")
			
			dictionary["HWTOPICSPUB"].append(d)


		## include standard topic messages in ROSMSG to force macro generation
		for r in [_ for _ in thread.resources if _.type == "rosmsg"]:
			log.debug("Include message:")
			log.debug(r.args)
			msgtype = r.args[0] + "/" + r.args[2]
			log.debug(msgtype)
			my_msg = msg_lib[msgtype]
			temp = mp.build_msg_dict(my_msg, msg_lib, mp.primitive_lib)
			if(type(temp == dict)):
				try:
					log.debug(dictionary["ROSMSG"].index(temp))
				except ValueError:
					dictionary["ROSMSG"].append(temp)

				


		log.info("Generating temporary HLS project in " + tmp.name + " ...")
		prj.apply_template("thread_hls_build", dictionary, tmp.name)

		log.info("Starting Vivado HLS ...")

		if  prj.impinfo.hls[1] != "2021.2" and prj.impinfo.hls[1] != "2022.1":
			subprocess.call("""
				source /opt/Xilinx/Vivado/{1}/settings64.sh;
				cd {0};
				vivado_hls -f script_csynth.tcl;
				vivado -mode batch -notrace -nojournal -nolog -source script_vivado_edn.tcl;""".format(tmp.name, prj.impinfo.hls[1]),
				shell=True, executable="/bin/bash")
		else:
				subprocess.call("""
				source /opt/Xilinx/Vivado/{1}/settings64.sh;
				cd {0};
				vitis_hls -f script_csynth.tcl;""".format(tmp.name, prj.impinfo.hls[1]),
				shell=True, executable="/bin/bash")

		dictionary = {}
		dictionary["NAME"] = thread.name.lower()
		dictionary["MEM"] = thread.mem
		dictionary["MEM_N"] = not thread.mem
		dictionary["HWSOURCE"] = thread.hwsource

		dictionary["HWTOPICSSUB"] = []
		for r in [_ for _ in thread.resources if _.type == "hwtopicsub"]:
			d = {}
			d["Name"] = r.name.replace("/", "")
			dictionary["HWTOPICSSUB"].append(d)

		dictionary["HWTOPICSPUB"] = []
		for r in [_ for _ in thread.resources if _.type == "hwtopicpub"]:
			d = {}
			d["Name"] = r.name.replace("/", "")
			dictionary["HWTOPICSPUB"].append(d)

		if(len(dictionary["HWTOPICSPUB"]) != 0 or  len(dictionary["HWTOPICSSUB"]) != 0):
			dictionary["RTIMPRESETDECLARATION"] = "ap_rst_n : in std_logic"
			dictionary["RTIMPRESETMAPPING"] = "ap_rst_n => rst_sig_n" 
			dictionary["RTIMPRESETREMAPPINGSIGNAL"] = "signal rst_sig_n : std_logic;"
			dictionary["RTIMPRESETREMAPPING"] = "rst_sig_n <= not HWT_Rst;"
		else:
			dictionary["RTIMPRESETREMAPPINGSIGNAL"] = ""
			dictionary["RTIMPRESETREMAPPING"] = ""
			dictionary["RTIMPRESETDECLARATION"] = "ap_rst : in std_logic"
			dictionary["RTIMPRESETMAPPING"] = "ap_rst => HWT_Rst"

		srcs = shutil2.join(tmp.name, "hls", "sol", "syn", "vhdl")
		#HLS instantiates subcores (e.g. floating point units) in VHDL form during the export step
		#The path above contains only .tcl instantiations, which our IP Packager flow doesn't understand
		#So we add extract the convenient .vhd definitions from the following path:
		srcs2 = shutil2.join(tmp.name, "hls", "sol", "impl", "ip", "hdl", "ip")
		dictionary["SOURCES"] = [srcs, srcs2]
		incls = shutil2.listfiles(srcs, True)
		##add only if pr is not enabled
		if not reconf_of_slots_or:
			incls += shutil2.listfiles(srcs2, True)
			
		dictionary["INCLUDES"] = [{"File": shutil2.trimext(_)} for _ in incls]
		#dictionary["INCLUDES"] = [{"FilewithExtension": shutil2.trimext(_) + shutil2.getext(_)} for _ in [item for item in incls if shutil2.getext(item) != ".v" and shutil2.getext(item) != ".tcl"]]
		dictionary["INCLUDES"] = [{"FilewithExtension": shutil2.trimext(_) + shutil2.getext(_)} for _ in [item for item in incls if shutil2.getext(item) != ".tcl"]]
		#Template will change top module entity name to "rt_reconf" if PR flow is used for this HWT
		dictionary["RECONFIGURABLE"] = reconf_of_slots_or#  prj.impinfo.pr
		if prj.impinfo.hls[1] == "2021.2":
			dictionary["VIVADO"] = "2021"
		elif prj.impinfo.hls[1] == "2022.1":
			dictionary["VIVADO"] = "2022"	
		else:
			dictionary["VIVADO"] = "2020"	

		log.info("Generating export files ...")
		if prj.impinfo.cpuarchitecture == "arm64":
			prj.apply_template("thread_hls_pcore_vhdl_64", dictionary, hwdir)
		else:
			prj.apply_template("thread_hls_pcore_vhdl", dictionary, hwdir)
		
		#copy ip tree
		ip_path = shutil2.join(tmp.name, "hls", "sol", "impl", "ip", "tmp.srcs", "sources_1", "ip")

		if(shutil2.exists(ip_path) and reconf_of_slots_or):
			ips = shutil2.listdirs(ip_path)
			shutil2.copytree(ip_path, shutil2.join(hwdir, "rt_{}_v1_00_a".format(thread.name.lower()) , "ip"))


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

def _export_hw_rosgateway_vivado(prj, hwdir, link, gateway):
	''' 
	Generates sources for one hardware thread for ReconOS in a Vivado project.
	
	It checks whether vhdl or hls sources shall be used and generates the hardware thread
	from the source templates. 
	
	hwdir gives the name of the project directory
	link boolean; if true files will be linked instead of copied
	thread is the name of the hardware thread to generate
	'''
	hwdir = hwdir if hwdir is not None else prj.basedir + ".hw" + "." + gateway.lower()

	log.info("Exporting rosgateway " + gateway + " to directory '" + hwdir + "'")

	gateways = [_ for _ in prj.threads if _.name == gateway]
	if (len(gateways) == 1):
		gateway = gateways[0]

	else:
		log.error("ROSGateway '" + gateway  + "' not found")
		return


	tmp = tempfile.TemporaryDirectory()

	dictionary = {}
	dictionary["PART"] = prj.impinfo.part
	dictionary["NAME"] = gateway.name.lower()
	dictionary["CLKPRD"] = min([_.clock.get_periodns() for _ in gateway.slots])

	if prj.impinfo.hls[1] == "2021.2":
		dictionary["VIVADO"] = "2021"
	elif prj.impinfo.hls[1] == "2022.1":
		dictionary["VIVADO"] = "2022"	
	else:
		dictionary["VIVADO"] = "2020"	
	
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

	
	srcs = shutil2.join(prj.dir, "src", "rosgateway_" + gateway.name.lower())
	
	dictionary["SOURCES"] = [srcs]
	files = shutil2.listfiles(srcs, True)
	dictionary["FILES"] = [{"File": _} for _ in files]
	dictionary["RESOURCES"] = []
	for i, r in enumerate([_ for _ in gateway.resources if (_.type != "hwtopicsub") and (_.type != "hwtopicpub")]):
		d = {}
		d["NameUpper"] = (r.group + "_" + r.name).upper()
		d["NameLower"] = (r.group + "_" + r.name).lower()
		d["LocalId"] = i
		d["HexLocalId"] =  "%08x" % i
		d["Type"] = r.type
		d["TypeUpper"] = r.type.upper()
		dictionary["RESOURCES"].append(d)

	dictionary["HWTOPIC"] = gateway.hwtopic.lower()
	dictionary["RESOURCEGROUP"] = (gateway.name + "_ressourcegroup").lower()



	dictionary["ROSMSG"] = []

	# parsing ROS message definitions
	msglib_base_path = os.getenv('RECONOS') + "/lib/ros_msgs"
	paths = [msglib_base_path + "/common_interfaces", msglib_base_path + "/rcl_interfaces"]
	msg_lib = mp.parse_msg_lib(paths)
	if(len(msg_lib) > 0):
		log.debug("-------------------------------Found msgs------------------------------")
		log.debug(msg_lib.keys())
	else:
		log.debug("Did not find any ROS-message definitions in the following paths:")
		for p in paths:
			log.debug(p)

	# build msg dictionary for subscriber of hwtopic
	dictionary["HWTOPICSSUB"] = []
	dictionary["HWPUBLISHERID"] = str(gateway.id)
	
	d = {}
	d["Name"] = gateway.hwtopic + "_in"
	d["NameUpper"] = d["Name"].replace("/", "").upper()
	d["HWPUBLISHERID"] = str(gateway.id)
	my_msg = msg_lib[gateway.msgtype]
	temp = mp.build_msg_dict(my_msg, msg_lib, mp.primitive_lib)
	if(type(temp == dict)):
		log.debug("----- Sucessfully build msg dict for subscriber -----")
		log.debug("-----------------------------------------------------")
		log.debug(temp)
		d.update(temp)
		dictionary["ROSMSG"].append(temp)
	else:
		log.error("Could not build msg dict for subscriber of hwtopic")

	dictionary["HWTOPICSSUB"].append(d)

	# build msg dict for publisher of hwtopic
	dictionary["HWTOPICSPUB"] = []

	d = {}
	d["Name"] = gateway.hwtopic + "_out"
	d["NameUpper"] = d["Name"] .replace("/", "").upper()
	d["HWPUBLISHERID"] = str(gateway.id)
	my_msg = msg_lib[gateway.msgtype]
	temp = mp.build_msg_dict(my_msg, msg_lib, mp.primitive_lib)
	if(type(temp == dict)):
		log.debug("----- sucessfully build msg dict for publisher ------")
		log.debug("-----------------------------------------------------")
		log.debug(temp)
		d.update(temp)
		#dictionary["ROSMSG"].append(temp)
	else:
		log.error("Could not build msg dict for publisher of hwtopic")
	
	dictionary["HWTOPICSPUB"].append(d)



	log.info("Generating temporary HLS project in " + tmp.name + " ...")
	prj.apply_template("rosgateway_hls_build", dictionary, tmp.name)

	log.info("Starting Vivado HLS ...")

	if  prj.impinfo.hls[1] != "2021.2" and prj.impinfo.hls[1] != "2022.1":
		subprocess.call("""
			source /opt/Xilinx/Vivado/{1}/settings64.sh;
			cd {0};
			vivado_hls -f script_csynth.tcl;
			vivado -mode batch -notrace -nojournal -nolog -source script_vivado_edn.tcl;""".format(tmp.name, prj.impinfo.hls[1]),
			shell=True, executable="/bin/bash")
	else:
			subprocess.call("""
			source /opt/Xilinx/Vivado/{1}/settings64.sh;
			cd {0};
			vitis_hls -f script_csynth.tcl;""".format(tmp.name, prj.impinfo.hls[1]),
			shell=True, executable="/bin/bash")

	dictionary = {}
	dictionary["NAME"] = gateway.name.lower()
	dictionary["HWSOURCE"] = "hls"

	dictionary["HWTOPICSSUB"] = []
	d = {}
	d["Name"] = gateway.hwtopic + "_in"
	dictionary["HWTOPICSSUB"].append(d)

	dictionary["HWTOPICSPUB"] = []
	d = {}
	d["Name"] = gateway.hwtopic + "_out"
	dictionary["HWTOPICSPUB"].append(d)

	if(len(dictionary["HWTOPICSPUB"]) != 0 or  len(dictionary["HWTOPICSSUB"]) != 0):
		dictionary["RTIMPRESETDECLARATION"] = "ap_rst_n : in std_logic"
		dictionary["RTIMPRESETMAPPING"] = "ap_rst_n => rst_sig_n" 
		dictionary["RTIMPRESETREMAPPINGSIGNAL"] = "signal rst_sig_n : std_logic;"
		dictionary["RTIMPRESETREMAPPING"] = "rst_sig_n <= not HWT_Rst;"
	else:
		dictionary["RTIMPRESETREMAPPINGSIGNAL"] = ""
		dictionary["RTIMPRESETREMAPPING"] = ""
		dictionary["RTIMPRESETDECLARATION"] = "ap_rst : in std_logic"
		dictionary["RTIMPRESETMAPPING"] = "ap_rst => HWT_Rst"

	srcs = shutil2.join(tmp.name, "hls", "sol", "syn", "vhdl")
	#HLS instantiates subcores (e.g. floating point units) in VHDL form during the export step
	#The path above contains only .tcl instantiations, which our IP Packager flow doesn't understand
	#So we add extract the convenient .vhd definitions from the following path:
	srcs2 = shutil2.join(tmp.name, "hls", "sol", "impl", "ip", "hdl", "ip")
	dictionary["SOURCES"] = [srcs, srcs2]
	incls = shutil2.listfiles(srcs, True)
	##add only if pr is not enabled
	incls += shutil2.listfiles(srcs2, True)
		
	dictionary["INCLUDES"] = [{"File": shutil2.trimext(_)} for _ in incls]
	#dictionary["INCLUDES"] = [{"FilewithExtension": shutil2.trimext(_) + shutil2.getext(_)} for _ in [item for item in incls if shutil2.getext(item) != ".v" and shutil2.getext(item) != ".tcl"]]
	dictionary["INCLUDES"] = [{"FilewithExtension": shutil2.trimext(_) + shutil2.getext(_)} for _ in [item for item in incls if shutil2.getext(item) != ".tcl"]]
	#Template will change top module entity name to "rt_reconf" if PR flow is used for this HWT
	dictionary["RECONFIGURABLE"] = False
	if prj.impinfo.hls[1] == "2021.2":
		dictionary["VIVADO"] = "2021"
	elif prj.impinfo.hls[1] == "2022.1":
		dictionary["VIVADO"] = "2022"	
	else:
		dictionary["VIVADO"] = "2020"	

	log.info("Generating export files ...")
	if prj.impinfo.cpuarchitecture == "arm64":
		prj.apply_template("thread_hls_pcore_vhdl_64", dictionary, hwdir)
	else:
		prj.apply_template("thread_hls_pcore_vhdl", dictionary, hwdir)
	
	
	#Save temporary HLS project directory for analysis:
	shutil2.mkdir("/tmp/ReconROS_hlsexport")
	save_dir_hls_prj = shutil2.join(hwdir, "..", "tmp_hls_prj_rosgateway_" + gateway.name.lower())
	shutil2.copytree(tmp.name, "/tmp/ReconROS_hlsexport")
	shutil2.rmtree(save_dir_hls_prj)
	shutil2.mkdir(save_dir_hls_prj)
	shutil2.copytree(tmp.name, save_dir_hls_prj)

		
def _export_hw_vivado(prj, hwdir, link):
	''' 
	Generates a TCL script for generation of a Vivado based project.
	
	It first compiles the configuration dictionary and then processes the templates
	according to the configuration dictionary.
	
	hwdir gives the name of the project directory
	link boolean; if true files will be linked instead of copied
	'''
	
	log.debug("export_hw_vivado")
	hwdir = hwdir if hwdir is not None else prj.basedir + ".hw"

	log.info("Export hardware to directory '" + hwdir + "'")

	dictionary = get_dict(prj)

	log.info("Generating export files ...")
	
	
	tmpl = "ref_" + prj.impinfo.os + "_" + "_".join(prj.impinfo.board) + "_" + prj.impinfo.design + "_" + prj.impinfo.xil[0] + "_" + prj.impinfo.xil[1]
	log.debug("Using template directory " + tmpl)
	if not shutil2.exists(prj.get_template(tmpl)):
		log.error("Template directory not found in project or ReconOS repository")
		return
	prj.apply_template(tmpl, dictionary, hwdir, link)

	log.info("Generating threads ...")

	log.debug("Start export of {} ROS gateways".format(len(prj.rosgateways)))
	for t in prj.threads:
		if hasattr(t, 'hwtopic'):
			export_hw_rosgateway(prj, shutil2.join(hwdir, "pcores"), link, t.name)


	log.debug("Start export of {} threads".format(len(prj.threads)))

	if (prj.impinfo.hlsmultithreading): 
		workerthreads = []


		for t in prj.threads:
			if not hasattr(t, 'hwtopic'):
				th = Thread(target=export_hw_thread, args=(prj, shutil2.join(hwdir, "pcores"), link, t.name))
				th.start()
				workerthreads.append(th)
			
		for th in workerthreads:
				th.join()

	else:
		for t in prj.threads:
			if not hasattr(t, 'hwtopic'):
				export_hw_thread(prj, shutil2.join(hwdir, "pcores"), link, t.name)

	

	if(shutil2.exists(shutil2.join(prj.dir, "ip_repo"))):
		shutil2.copytree(shutil2.join(prj.dir, "ip_repo"), shutil2.join(hwdir, "pcores"))
		log.debug("Copy pcores from {} to {} folder".format(shutil2.join(prj.dir, "ip_repo"), shutil2.join(hwdir, "pcores") ))
	else:
		log.debug("no ip_repo folder in {} found".format(shutil2.join(prj.dir, "ip_repo")))
		
	log.info("Calling TCL script to generate Vivado IP Repository")
	result = subprocess.call("""
					source /opt/Xilinx/Vivado/{1}/settings64.sh;
					cd {0};
					vivado -mode batch -notrace -nojournal -nolog -source create_ip_library.tcl;""".format(hwdir, prj.impinfo.xil[1]),
					shell=True,executable="/bin/bash")
	if result != 0 :
		log.critical("Generation of Vivado IP repository failed. Maybe you specified unknown components in build.cfg?")
		exit(1)
	
	log.info("Calling TCL script to generate ReconOS in Vivado IP Integrator")
	subprocess.call("""
					source /opt/Xilinx/Vivado/{1}/settings64.sh;
					cd {0};
					vivado -mode batch -notrace -nojournal -nolog -source export.tcl -tclargs -proj_name myReconOS -proj_path . ;""".format(hwdir, prj.impinfo.xil[1]),
					shell=True,  executable="/bin/bash")

