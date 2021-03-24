import logging
import argparse

log = logging.getLogger(__name__)

def get_cmd(prj):
	return "info"

def get_call(prj):
	return info_cmd

def get_parser(prj):
	parser = argparse.ArgumentParser("info", description="""
		Prints informations regarding the active project.
		""")
	return parser

def info_cmd(args):
	info(args)

def info(args):
	prj = args.prj
	print("-" * 40)
	print("ReconOS Project '" + prj.name + "'")
	print("  Board".ljust(20) + str(prj.impinfo.board))
	print("  Reference Design".ljust(20) + prj.impinfo.design)
	print("  Part".ljust(20) + prj.impinfo.part)
	print("  Operating System".ljust(20) + prj.impinfo.os)
	print("  Xilinx Tools".ljust(20) + ",".join(prj.impinfo.xil))
	print("  CFlags".ljust(20) + prj.impinfo.cflags)
	print("  LdFlags".ljust(20) + prj.impinfo.ldflags)
	print("-" * 40)
	print("Clocks:")
	for c in prj.clocks:
		print("  " + (c.name + "*" if c == prj.clock else "").ljust(18) + "[freq=" + str(c.freq // 1000000) + "MHz]")
	print("Slots:")
	for s in prj.slots:
		print("  " + s.name.ljust(18) + "[id=" + str(s.id) + ",clk=" + s.clock.name + "]")
	print("Resources:")
	for r in prj.resources:
		print("  " + r.name.ljust(18) + "[id=" + str(r.id) + ",type=" + r.type + ",args=" + str(r.args) + ",group=" + r.group + "]")
	print("Threads:")
	for t in prj.threads:
		print("  " + t.name.ljust(18) + "[slots=" + str(t.slots) + ",resources=" + str(t.resources) + "]")