import reconos.utils.shutil2 as shutil2
import reconos.utils.template as template

import logging
import argparse
import subprocess

log = logging.getLogger(__name__)

def get_cmd(prj):
	return "deploy_run"

def get_call(prj):
	return deploy_run_cmd

def get_parser(prj):
	parser = argparse.ArgumentParser("deploy_run", description="""
		Runs the build outputs on the compute nodes
		""")
	#parser.add_argument("hwdir", help="alternative export directory", nargs="?")
	return parser

def deploy_run_cmd(args):
	print("deploy_run cmd")