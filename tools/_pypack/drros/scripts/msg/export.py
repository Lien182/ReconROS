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
	print("called export_msg")


	

