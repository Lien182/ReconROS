import reconos.utils.shutil2 as shutil2
import reconos.utils.template as template

import logging
import argparse
import subprocess

log = logging.getLogger(__name__)

def get_cmd(prj):
	return "deploy_hw"

def get_call(prj):
	return deploy_cmd

def get_parser(prj):
	parser = argparse.ArgumentParser("deploy_hw", description="""
		Deploys the hardware bitstream on the target hardware""")

	parser.add_argument("targetboardaddress",  help="alternative board address", nargs="?")    
	parser.add_argument("targetboarduser", help="alternative user name", nargs="?")
	parser.add_argument("targetboardpassword", help="alternative password", nargs="?")

	return parser

def deploy_cmd(args):
	deploy(args.prj, args.targetboardaddress, args.targetboarduser, args.targetboardpassword)

def deploy(prj, targetboard, targetboarduser, targetboardpassword):
	if prj.impinfo.targetboardaddress != None:
		targetboardaddress = prj.impinfo.targetboardaddress

	if prj.impinfo.targetboarduser != None:
		targetboarduser = prj.impinfo.targetboarduser

	if prj.impinfo.targetboardpassword != None:
		targetboardpassword = prj.impinfo.targetboardpassword

	subprocess.call('sshpass -p "{}" scp build.hw/myReconOS.runs/impl_1/design_1_wrapper.bin {}@{}:/home/xilinx/'.format(targetboardpassword, targetboarduser, targetboardaddress), shell=True,  executable="/bin/bash")


