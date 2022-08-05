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

	if prj.impinfo.pr:
		subprocess.call('sshpass -p "{}" scp build.hw/Bitstreams/Config_reconf_1_*.bin {}@{}:/mnt/design_1_wrapper.bin'.format(targetboardpassword, targetboarduser, targetboardaddress), shell=True,  executable="/bin/bash")
		subprocess.call('sshpass -p "{}" ssh {}@{} "rm -f -r /mnt/bitstreams/*"'.format(targetboardpassword, targetboarduser, targetboardaddress), shell=True,  executable="/bin/bash")
		subprocess.call('sshpass -p "{}" scp build.hw/Bitstreams/pblock_* {}@{}:/mnt/bitstreams/'.format(targetboardpassword, targetboarduser, targetboardaddress), shell=True,  executable="/bin/bash")

	else:
		subprocess.call('sshpass -p "{}" scp build.hw/myReconOS.runs/impl_1/design_1_wrapper.bin {}@{}:/mnt/'.format(targetboardpassword, targetboarduser, targetboardaddress), shell=True,  executable="/bin/bash")


