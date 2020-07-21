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
	parser = argparse.ArgumentParser("build_hw", description="""
		Builds the hardware project and generates a bitstream to
		be downloaded to the FPGA.
		""")
	parser.add_argument("hwdir", help="alternative export directory", nargs="?")
	return parser

def build_cmd(args):
	build(args.prj, args.hwdir)


def build(prj, msgdir):
	msgdir = prj.basedir + ".msg"

	#try:
	#	shutil2.chdir(swdir)
	#except:
	#	log.error("software directory '" + swdir + "' not found")
	#	return
	
	subprocess.call("bash $RECONOS/tools/arm-docker-build/build_msg.sh", shell=True)

	print()
	shutil2.chdir(prj.dir)
