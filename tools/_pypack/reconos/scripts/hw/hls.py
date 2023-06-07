import reconos.utils.shutil2 as shutil2
import reconos.utils.template as template
import reconos.scripts.hw.export

import logging
import argparse

import tempfile

log = logging.getLogger(__name__)

def get_cmd(prj):
	return "export_hls"

def get_call(prj):
	return export_hls_cmd

def get_parser(prj):
	parser = argparse.ArgumentParser("export_hls", description="""
		Creates a hls project to analyse a single hardware thread
		(FIXME: Solution not created and therefore directives missing)
		""")
	parser.add_argument("thread", help="ReconOS thread to create testbench for")
	parser.add_argument("-l", "--link", help="link sources instead of copying", action="store_true")
	parser.add_argument("hlsdir", help="alternative export directory", nargs="?")
	return parser

def export_hls_cmd(args):
	export_hls(args, args.hlsdir, args.link, args.thread)


def export_hls(args, hlsdir, link, thread):
	log.error("Xilinx tool not supported")