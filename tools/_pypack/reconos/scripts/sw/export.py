import reconos.utils.custom_msgs as custom_msgs
import reconos.utils.shutil2 as shutil2
import reconos.utils.template as template

import logging
import argparse

import shutil
import os
import subprocess
from os import listdir
from os.path import isfile, join

import re


import re

log = logging.getLogger(__name__)

def get_cmd(prj):
	return "export_sw"

def get_call(prj):
	return export_sw_cmd

def get_parser(prj):
	parser = argparse.ArgumentParser("export_hw", description="""
		Exports the software project and generates all necessary files.
		""")
	parser.add_argument("-l", "--link", help="link sources instead of copying", default=False, action="store_true")
	parser.add_argument("-t", "--thread", help="export only single thread")
	parser.add_argument("swdir", help="alternative export directory", nargs="?")
	return parser

def export_sw_cmd(args):
	if (args.thread is None):
		export_sw(args, args.swdir, args.link)
	else:
		export_sw_thread(args, args.swdir, args.link, args.thread)

def export_sw(args, swdir, link):
	prj = args.prj
	swdir = swdir if swdir is not None else prj.basedir + ".sw"

	log.info("Export software to project directory '" + prj.dir + "'")

	dictionary = {}
	dictionary["NAME"] = prj.name.lower()
	dictionary["CFLAGS"] = prj.impinfo.cflags
	dictionary["LDFLAGS"] = prj.impinfo.ldflags
	dictionary["ROS2_DISTRIBUTION"] = prj.impinfo.ros2distribution[:1].upper()
	dictionary["ROS2_DISTRO"] = prj.impinfo.ros2distribution
	dictionary["THREADS"] = []
	dictionary["ROSMsgHeader"] = ""
	dictionary["MSGINCLUDEDIR"] = " ".join([
		"-I " + shutil2.relpath(msg_include_dir, swdir)
		for msg_include_dir in custom_msgs.get_absolute_include_paths(prj.dir)
	])

	if prj.impinfo.cpuarchitecture == "arm64":
		dictionary["RRBASETYPE"] 		= "int64_t"
		dictionary["RRUBASETYPE"] 		= "uint64_t"
		dictionary["RRBASETYPEBYTES"] 	= "8"
	elif prj.impinfo.cpuarchitecture == "x86_64":
		dictionary["RRBASETYPE"] 		= "int64_t"
		dictionary["RRUBASETYPE"] 		= "uint64_t"
		dictionary["RRBASETYPEBYTES"] 	= "8"
	else:
		dictionary["RRBASETYPE"] 		= "int32_t"
		dictionary["RRUBASETYPE"] 		= "uint32_t"
		dictionary["RRBASETYPEBYTES"] 	= "4"


	for t in prj.threads:
		d = {}
		d["Name"] = t.name.lower()
		d["Slots"] = ",".join([str(_.id) for _ in t.slots])
		d["SlotCount"] = len(t.slots)
		d["Resources"] = ",".join(["&" + (_.group + "_" + _.name).lower() + "_res" for _ in t.resources if not (_.type=="hwtopic" or _.type=="hwtopicpub" or _.type=="hwtopicsub")])
		d["ResourceCount"] = len([t.resources for _ in t.resources if not (_.type=="hwtopic" or _.type=="hwtopicpub" or _.type=="hwtopicsub")])
		d["HasHw"] = t.hwsource is not None
		d["HasSw"] = t.swsource is not None
		dictionary["THREADS"].append(d)
	dictionary["RESOURCES"] = []
	dictionary["RESOURCEGROUPS"] = []
	for r in prj.resources:
		d = {}
		
		d["Group"] = r.group.lower()
		d["Id"] = r.id
		d["NameUpper"] = (r.group + "_" + r.name).upper()
		d["NameLower"] = (r.group + "_" + r.name).lower()
		d["Type"] = r.type
		d["TypeUpper"] = r.type.upper()
		if r.type == "rossub":
			for msg in prj.resources:
				if (msg.name == r.args[1]) and (msg.group == r.group):
					d["Args"] = r.args[0] + "," + "rosidl_typesupport_c__get_message_type_support_handle__" + msg.args[0] +"__"+ msg.args[1] +"__"+ msg.args[2].replace('_', '') +"(), " + r.args[2]+ ", " + r.args[3] 
					#print(d["Args"])
					break

		elif r.type == "rospub":
			for msg in prj.resources:
				if msg.name == r.args[1] and (msg.group == r.group):
					d["Args"] = r.args[0] + "," + "rosidl_typesupport_c__get_message_type_support_handle__" + msg.args[0] +"__"+ msg.args[1] +"__"+ msg.args[2].replace('_', '') +"(), " + r.args[2] 
					break

		elif r.type == "rossrvs":
			for msg in prj.resources:
				#print(msg.name.replace('_req','') + ";" +r.args[1] + ";")
				if msg.name.replace('_req','') == r.args[1]:
					d["Args"] = r.args[0] + "," + "rosidl_typesupport_c__get_service_type_support_handle__" + msg.args[0] +"__"+ msg.args[1] +"__"+ msg.args[2] +"(), " + r.args[2] + ", " + r.args[3] 
					#print(d["Args"])
					break
		elif r.type == "rossrvc":
			for msg in prj.resources:
				#print(msg.name.replace('_req','') + ";" +r.args[1] + ";")
				if msg.name.replace('_req','') == r.args[1]:
					d["Args"] = r.args[0] + "," + "rosidl_typesupport_c__get_service_type_support_handle__" + msg.args[0] +"__"+ msg.args[1] +"__"+ msg.args[2] +"(), " + r.args[2] + ", " + r.args[3] 
					#print(d["Args"])
					break
		elif r.type == "rosactions":
			for msg in prj.resources:
				#print(msg.name.replace('_goal_req','') + ";" +r.args[1] + ";")
				if msg.name.replace('_goal_req','') == r.args[1]:
					d["Args"] = r.args[0] + "," + "rosidl_typesupport_c__get_action_type_support_handle__" + msg.args[0] +"__"+ msg.args[1] +"__"+ msg.args[2] +"(), " + r.args[2] + ", " + r.args[3] 
					#print(d["Args"])
					break
		elif r.type == "rosactionc":
			for msg in prj.resources:
				#print(msg.name.replace('_goal_req','') + ";" +r.args[1] + ";")
				if msg.name.replace('_goal_req','') == r.args[1]:
					d["Args"] = r.args[0] + "," + "rosidl_typesupport_c__get_action_type_support_handle__" + msg.args[0] +"__"+ msg.args[1] +"__"+ msg.args[2] +"(), " + r.args[2] + ", " + r.args[3] 
					#print(d["Args"])
					break
		else:
			if not (r.type=="hwtopic" or r.type=="hwtopicpub" or r.type=="hwtopicsub"):
				#print(r.type +";" +  str(r.args))
				d["Args"] = ", ".join(r.args)
				
		d["Id"] = r.id
		if r.type == "rosmsg":
			if len(r.args) == 3:
				#problem of ros2 dashing
				#naming of header files for uint are inconsistant
				fixedheader = r.args[2].lower().replace('_', '')
				#fixedheader = re.sub("*_*", "", r.args[2].lower(),1, re.MULTILINE | re.VERBOSE)

				d["ROSDataType"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2].replace('_', '')
				d["ROSDataTypeInitFunc"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2].replace('_', '') + "__create"
				d["ROSDataTypeDeInitFunc"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2].replace('_', '') + "__destroy"
				d["ROSDataTypeSequenceLength"] = " "
				dictionary["ROSMsgHeader"] += ("#include <" + r.args[0] +"/"+ r.args[1] +"/"+ r.args[2].lower() + ".h>\n").lower()
				#print(("#include <" + r.args[0] +"/"+ r.args[1] +"/"+ r.args[2].lower() + ".h>\n").lower())
			elif len(r.args) == 4:
				d["ROSDataType"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2]+"__Sequence"
				d["ROSDataTypeInitFunc"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2] +"__Sequence__create"
				d["ROSDataTypeDeInitFunc"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2]+"__Sequence__destroy"
				d["ROSDataTypeSequenceLength"] = r.args[3]
		if r.type == "rossrvmsgreq":
			if len(r.args) == 3: 
				d["ROSDataType"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2]+ "_Request"
				d["ROSDataTypeInitFunc"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2] + "_Request" + "__create"
				d["ROSDataTypeDeInitFunc"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2] + "_Request" + "__destroy"
				d["ROSDataTypeSequenceLength"] = " "
				dictionary["ROSMsgHeader"] += ("#include <" + r.args[0] +"/"+ r.args[1] +"/"+ r.args[2] + ".h>\n").lower()
		if r.type == "rossrvmsgres":
			if len(r.args) == 3: 
				d["ROSDataType"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2]+ "_Response"
				d["ROSDataTypeInitFunc"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2] + "_Response" + "__create"
				d["ROSDataTypeDeInitFunc"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2] + "_Response" + "__destroy"
				d["ROSDataTypeSequenceLength"] = " "

			
		if r.type == "rosactionmsggoalreq":
			if len(r.args) == 3: 
				d["ROSDataType"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2]+ "_SendGoal_Request"
				d["ROSDataTypeInitFunc"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2] + "_SendGoal_Request" + "__create"
				d["ROSDataTypeDeInitFunc"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2] + "_SendGoal_Request" + "__destroy"
				d["ROSDataTypeSequenceLength"] = " "
				dictionary["ROSMsgHeader"] += ("#include <" + r.args[0] +"/"+ r.args[1] +"/"+ r.args[2] + ".h>\n").lower()
		if r.type == "rosactionmsggoalres":
			if len(r.args) == 3: 
				d["ROSDataType"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2]+ "_Response"
				d["ROSDataTypeInitFunc"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2] + "_Response" + "__create"
				d["ROSDataTypeDeInitFunc"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2] + "_Response" + "__destroy"
				d["ROSDataTypeSequenceLength"] = " "

		if r.type == "rosactionmsgresultreq":
			if len(r.args) == 3: 
				d["ROSDataType"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2]+ "_Request"
				d["ROSDataTypeInitFunc"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2] + "_Request" + "__create"
				d["ROSDataTypeDeInitFunc"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2] + "_Request" + "__destroy"
				d["ROSDataTypeSequenceLength"] = " "	

		if r.type == "rosactionmsgresultres":
			if len(r.args) == 3: 
				d["ROSDataType"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2]+ "_GetResult_Response"
				d["ROSDataTypeInitFunc"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2] + "_GetResult_Response" + "__create"
				d["ROSDataTypeDeInitFunc"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2] + "_GetResult_Response" + "__destroy"
				d["ROSDataTypeSequenceLength"] = " "

		if r.type == "rosactionmsgfeedback":
			if len(r.args) == 3: 
				d["ROSDataType"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2]+ "_FeedbackMessage"
				d["ROSDataTypeInitFunc"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2] + "_FeedbackMessage" + "__create"
				d["ROSDataTypeDeInitFunc"] = r.args[0] +"__"+ r.args[1] +"__"+ r.args[2] + "_FeedbackMessage" + "__destroy"
				d["ROSDataTypeSequenceLength"] = " "

		if not (r.type=="hwtopic" or r.type=="hwtopicpub" or r.type=="hwtopicsub"):
			dictionary["RESOURCES"].append(d)
		
		#dictionary["RESOURCES"].append(d)


		for idx in range(len(dictionary["RESOURCEGROUPS"])):
			if dictionary["RESOURCEGROUPS"][idx]["Name"] == r.group.lower():
				dictionary["RESOURCEGROUPS"][idx]["Items"].append(d)
				break
		else:
			e = {}
			e["Name"] = r.group.lower()
			e["Items"]= []
			e["Items"].append(d)
			#print(r.group.lower())
			dictionary["RESOURCEGROUPS"].append(e)
			
	
	log.debug("RESOURCEGROUPS")
	log.debug(dictionary["RESOURCEGROUPS"])

	dictionary["CLOCKS"] = []
	for c in prj.clocks:
		d = {}
		d["NameLower"] = c.name.lower()
		d["Id"] = c.id
		param = c.get_pllparam(800000000, 1600000000, 100000000)
		d["M"] = param[0]
		d["O"] = param[1]
		dictionary["CLOCKS"].append(d)

	srcs = shutil2.join(prj.dir, "src", "application")
	dictionary["SOURCES"] = [srcs]

	log.info("Generating export files ...")
	templ = "app_" + prj.impinfo.os
	prj.apply_template(templ, dictionary, swdir, link)

	log.info("Generating threads ...")
	for t in prj.threads:
		export_sw_thread(args, swdir, link, t.name)

	dictionary = {}
	dictionary["OS"] = prj.impinfo.os.lower()
	dictionary["BOARD"] = "_".join(prj.impinfo.board)
	dictionary["REPO_REL"] = shutil2.relpath(prj.impinfo.repo, swdir)
	dictionary["OBJS"] = [{"Source": shutil2.trimext(_) + ".o"}
	                       for _ in shutil2.listfiles(swdir, True, "c[cp]*$")]

	template.preproc(shutil2.join(swdir, "Makefile"), dictionary, "overwrite", force=True)

	
	
def export_sw_thread(args, swdir, link, thread):
	prj = args.prj
	swdir = swdir if swdir is not None else prj.basedir + ".sw" + "." + thread.lower()

	log.info("Exporting thread " + thread + " to directory '" + swdir + "'")

	threads = [_ for _ in prj.threads if _.name == thread]
	if (len(threads) == 1):
		thread = threads[0]

		if thread.swsource is None:
			log.info("No software source specified")
			return
	else:
		log.info("Thread '" + thread  + "' not found")
		return

	dictionary = {}
	dictionary["NAME"] = thread.name.lower()
	dictionary["RESOURCES"] = []
	for i,r in enumerate(thread.resources):
		d = {}
		d["NameUpper"] = (r.group + "_" + r.name).upper()
		d["NameLower"] = (r.group + "_" + r.name).lower()
		d["Id"] = r.id
		d["HexId"] = "%08x" % r.id
		d["LocalId"] = i
		d["HexLocalId"] = "%08x" % i
		d["Type"] = r.type
		d["TypeUpper"] = r.type.upper()
		#dictionary["RESOURCES"].append(d)
		if not (r.type=="hwtopic" or r.type=="hwtopicpub" or r.type=="hwtopicsub"):
			dictionary["RESOURCES"].append(d)
	dictionary["SOURCES"] = [shutil2.join(prj.dir, "src", "rt_" + thread.name.lower(), thread.swsource)]

	log.info("Generating export files ...")
	if thread.swsource == "c":
		prj.apply_template("thread_c_plain", dictionary, swdir, link)
	elif thread.swsource == "hls":
		prj.apply_template("thread_c_hls", dictionary, swdir, link)
