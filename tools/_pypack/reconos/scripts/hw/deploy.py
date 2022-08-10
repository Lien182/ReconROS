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

	parser.add_argument("foldername", help="alternative name", nargs="?")
	parser.add_argument("targetboardaddress",  help="alternative board address", nargs="?")    
	parser.add_argument("targetboarduser", help="alternative user name", nargs="?")
	parser.add_argument("targetboardpassword", help="alternative password", nargs="?")
	
	

	return parser

def deploy_cmd(args):
	deploy(args.prj, args.targetboardaddress, args.targetboarduser, args.targetboardpassword, args.foldername)

def deploy(prj, targetboard, targetboarduser, targetboardpassword, foldername):
	if prj.impinfo.targetboardaddress != None:
		targetboardaddress = prj.impinfo.targetboardaddress

	if prj.impinfo.targetboarduser != None:
		targetboarduser = prj.impinfo.targetboarduser

	if prj.impinfo.targetboardpassword != None:
		targetboardpassword = prj.impinfo.targetboardpassword

	
	
	
	if 	foldername != None:
		
		path = "build.hw_{}".format(foldername)
	else:
		path = "build.hw"		

	print("Copy bitstream from {}".format(path))

	if prj.impinfo.pr:
		subprocess.call('sshpass -p "{}" scp {}/Bitstreams/Config_reconf_*.bin {}@{}:/mnt/design_1_wrapper.bin'.format(targetboardpassword, path, targetboarduser, targetboardaddress), shell=True,  executable="/bin/bash")
		subprocess.call('sshpass -p "{}" ssh {}@{} "rm -f -r /mnt/bitstreams/*"'.format(targetboardpassword, targetboarduser, targetboardaddress), shell=True,  executable="/bin/bash")
		subprocess.call('sshpass -p "{}" scp {}/Bitstreams/pblock_* {}@{}:/mnt/bitstreams/'.format(targetboardpassword, path, targetboarduser, targetboardaddress), shell=True,  executable="/bin/bash")

	else:
		subprocess.call('sshpass -p "{}" scp {}/myReconOS.runs/impl_1/design_1_wrapper.bin {}@{}:/mnt/'.format(targetboardpassword,path, targetboarduser, targetboardaddress), shell=True,  executable="/bin/bash")


