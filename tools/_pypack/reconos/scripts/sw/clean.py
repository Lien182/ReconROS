import reconos.utils.shutil2 as shutil2

import logging
import argparse
import subprocess

log = logging.getLogger(__name__)

def get_cmd(prj):
	return "clean_sw"

def get_call(prj):
	return clean_cmd

def get_parser(prj):
	parser = argparse.ArgumentParser("clean_sw", description="""
		Cleans the software project.
		""")
	parser.add_argument("-r", "--remove", help="remove entire software directory", action="store_true")
	return parser

def clean_cmd(args):
	clean(args)

def clean(args):
	prj = args.prj
	swdir = prj.basedir + ".sw"

	if args.remove:
		shutil2.rmtree(swdir)
	else:
		try:
			shutil2.chdir(swdir)
		except:
			log.error("software directory '" + swdir + "' not found")
			return
		
		subprocess.call("make clean", shell=True)

		print()
		shutil2.chdir(prj.dir)