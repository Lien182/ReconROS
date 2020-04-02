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
	if args.prj.impinfo.xil[0] == "ise":
		export_hls_ise(args, hlsdir, link, thread)
	else:
		log.error("Xilinx tool not supported")

def export_hls_ise(args, hlsdir, link, thread):
	prj = args.prj
	hlsdir = hlsdir if hlsdir is not None else prj.basedir + ".hls" + "." + thread.lower()

	log.info("Generating hls project for thread " + thread + " to '" + hlsdir + "'")

	threads = [_ for _ in prj.threads if _.name == thread]
	if (len(threads) == 1):
		thread = threads[0]

		if thread.hwsource is None or not thread.hwsource == "hls":
			log.error("No hardware source specified")
	else:
		log.error("Thread '" + thread  + "' not found")
		return

	dictionary = {}
	dictionary["PART"] = prj.impinfo.part
	dictionary["NAME"] = thread.name.lower()
	dictionary["CLKPRD"] = min([_.clock.get_periodns() for _ in thread.slots])
	srcs = shutil2.join(prj.dir, "src", "rt_" + thread.name.lower(), thread.hwsource)
	dictionary["SOURCES"] = [srcs]
	files = shutil2.listfiles(srcs, True)
	dictionary["FILES"] = [{"File": _} for _ in files]
	dictionary["HLSDIR"] = hlsdir
	dictionary["MEM"] = thread.mem
	dictionary["MEM_N"] = not thread.mem
	dictionary["RESOURCES"] = []
	for i, r in enumerate(thread.resources):
		d = {}
		d["Id"] = r.id
		d["NameUpper"] = (r.group + "_" + r.name).upper()
		d["NameLower"] = (r.group + "_" + r.name).lower()
		d["LocalId"] = i
		d["HexLocalId"] =  "%08x" % i
		d["Type"] = r.type
		d["TypeUpper"] = r.type.upper()
		dictionary["RESOURCES"].append(d)

	log.info("Generating export files ...")
	prj.apply_template("thread_hls_prj", dictionary, hlsdir)