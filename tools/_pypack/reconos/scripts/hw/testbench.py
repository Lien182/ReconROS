import reconos.utils.shutil2 as shutil2
import reconos.utils.template as template
import reconos.scripts.hw.export

import logging
import argparse

import tempfile

log = logging.getLogger(__name__)

def get_cmd(prj):
	return "export_sim"

def get_call(prj):
	return export_sim_cmd

def get_parser(prj):
	parser = argparse.ArgumentParser("export_sim", description="""
		Creates a testbench to simulate a single hardware thread.
		""")
	parser.add_argument("thread", help="ReconOS thread to create testbench for")
	parser.add_argument("-l", "--link", help="link sources instead of copying", action="store_true")
	parser.add_argument("simdir", help="alternative export directory", nargs="?")
	return parser

def export_sim_cmd(args):
	export_sim(args, args.simdir, args.link, args.thread)


def export_sim(args, simdir, link, thread):
	if args.prj.impinfo.xil[0] == "ise":
		export_sim_ise(args, simdir, link, thread)
	else:
		log.error("Xilinx tool not supported")

def export_sim_ise(args, simdir, link, thread):
	prj = args.prj
	simdir = simdir if simdir is not None else prj.basedir + ".sim" + "." + thread.lower()

	log.info("Generating testbench for thread " + thread + " to '" + simdir + "'")

	threads = [_ for _ in prj.threads if _.name == thread]
	if (len(threads) == 1):
		thread = threads[0]

		if thread.hwsource is None:
			log.error("No hardware source specified")
	else:
		log.error("Thread '" + thread  + "' not found")
		return

	tmp = tempfile.TemporaryDirectory()
	reconos.scripts.hw.export.export_hw_thread(args, tmp.name, link, thread.name)

	dictionary = {}
	dictionary["THREAD"] = thread.name.lower()
	srcs = shutil2.join(tmp.name, "rt_" + thread.name.lower() + "_v1_00_a", "hdl", "vhdl")
	dictionary["SOURCES"] = [srcs]
	files = shutil2.listfiles(srcs, True)
	dictionary["FILES"] = [{"File": _} for _ in files]

	log.info("Generating export files ...")
	tmpl = "sim_testbench_" + prj.impinfo.xil[1]
	prj.apply_template(tmpl, dictionary, simdir, link)