import reconos.utils.shutil2 as shutil2
import reconos.utils.template as template

import logging
import argparse
import subprocess

log = logging.getLogger(__name__)

def get_cmd(prj):
	return "build_msg"

def get_call(prj):
	return build_cmd

def get_parser(prj):
	parser = argparse.ArgumentParser("build_msg", description="""
		Builds the ROS messages project """)
	parser.add_argument("msgdir", help="alternative export directory", nargs="?")
	return parser

def build_cmd(args):
	build(args.prj, args.msgdir)


def build(prj, msgdir):
	msgdir = prj.basedir + ".msg"

	if not(prj.impinfo.cpuarchitecture == "x86" or prj.impinfo.cpuarchitecture == "x86_64"):
		subprocess.call("""bash $RECONOS/tools/arm-docker-build/build_msg.sh {0} {1}""".format(prj.impinfo.cpuarchitecture, prj.impinfo.ros2distribution), shell=True)
	else:
		try:
			shutil2.chdir(msgdir)
		except:
			log.error("software directory '" + msgdir + "' not found")
			return

		subprocess.call("source /opt/ros/dashing/setup.bash; colcon build; rm build -r -f", shell=True, executable="/bin/bash")



	print()
	shutil2.chdir(prj.dir)
