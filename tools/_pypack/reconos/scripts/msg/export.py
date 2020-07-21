import reconos.utils.shutil2 as shutil2
import reconos.utils.template as template

import logging
import argparse

import shutil
import os
import subprocess
from os import listdir
from os.path import isfile, join, isdir


log = logging.getLogger(__name__)

def get_cmd(prj):
	return "export_msg"

def get_call(prj):
	return export_msg_cmd

def get_parser(prj):
	parser = argparse.ArgumentParser("export_hw", description="""
		Exports the software project and generates all necessary files.
		""")
	parser.add_argument("-l", "--link", help="link sources instead of copying", default=False, action="store_true")
	parser.add_argument("-t", "--thread", help="export only single thread")
	parser.add_argument("swdir", help="alternative export directory", nargs="?")
	return parser

def export_msg_cmd(args):
	export_msg(args, args.swdir, args.link)
	
def export_msg(args, msgdir, link):
	prj = args.prj
	msgdir = msgdir if msgdir is not None else prj.basedir + ".msg"

	if shutil2.exists(msgdir):
		shutil.rmtree(msgdir)
	shutil2.mkdir(msgdir)

	log.info("Export software to project directory '" + prj.dir + "'")

	msg_packages = [f for f in listdir(prj.dir + "/msg/") if isdir(join(prj.dir + "/msg/", f))]

	print(msg_packages)

	for msgs_packs in msg_packages:
		dictionary = {}
		dictionary["pkgname"] = msgs_packs
		srv_files = {}
		if shutil2.exists(prj.dir + "/msg/"+msgs_packs+"/srv"):
			srv_files = [f for f in listdir(prj.dir + "/msg/"+msgs_packs+"/srv") if isfile(join(prj.dir + "/msg/"+msgs_packs+"/srv", f))]
			print(srv_files)
		msg_files = {}
		if shutil2.exists(prj.dir + "/msg/"+msgs_packs+"/msg/"):
			msg_files = [f for f in listdir(prj.dir + "/msg/"+msgs_packs+"/msg/") if isfile(join(prj.dir + "/msg/"+msgs_packs+"/msg/", f))]
			print(msg_files)

		dictionary["interface_files"] = ""
		for f in srv_files:
			dictionary["interface_files"] += '"srv/'+f+'" '
	
		for f in msg_files:
			dictionary["interface_files"] += '"msg/'+f+'" '


		print(dictionary["interface_files"])	
		prj.apply_template("ros_msg", dictionary, msgdir + "/"+ msgs_packs, link)
		if shutil2.exists(prj.dir + "/msg/" + msgs_packs + "/srv/"):
			shutil.copytree( prj.dir + "/msg/" + msgs_packs + "/srv/", msgdir + "/" + msgs_packs + "/srv/")
		if shutil2.exists(prj.dir + "/msg/" + msgs_packs + "/msg/"):	
			shutil.copytree( prj.dir + "/msg/" + msgs_packs + "/msg/", msgdir + "/" + msgs_packs + "/msg/")


	# #bashCommand = "source /opt/ros/foxy/setup.bash; cd "+swdir+ "/msg; ros2 pkg create my_reconros_services --dependencies builtin_interfaces"
	# #print(bashCommand)
	# #sp = subprocess.Popen(["/bin/sh", "-c",bashCommand],shell=False, executable="/bin/bash")
	# #sp.communicate()

	

