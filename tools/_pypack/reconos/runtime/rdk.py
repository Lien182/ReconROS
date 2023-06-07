#                                                        ____  _____
#                            ________  _________  ____  / __ \/ ___/
#                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
#                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
#                         /_/   \___/\___/\____/_/ /_/\____//____/
# 
# ======================================================================
# 
#   project:      ReconOS - Toolchain
#   author:       Christoph Rüthing, University of Paderborn
#   description:  Main toolchain script.
# 
# ======================================================================

import reconos.runtime.project as project
import reconos.utils.shutil2 as shutil2

import sys
import readline
import pkgutil
import importlib
import reconos
import reconos.scripts
import argparse
import logging

logging.basicConfig(format="[reconos-toolchain | %(name)s] %(levelname)s: %(message)s")
log = logging.getLogger(__name__)

#
# Main function implementing main loop of toolchain.
#
def main():
	prj = project.Project()
	prjfiles = shutil2.listfiles(".", ext="cfg$", rel=True)
	if not prjfiles:
		log.error("Project file not found")
		sys.exit(1)
	elif len(prjfiles) > 1:
		print("Multiple project files found. Please select:")
		for i,f in enumerate(prjfiles):
			print("  [" + str(i) + "] " + f)
		prjfile = prjfiles[int(input())]
		print("Selected '" + prjfile + "'")
	else:
		prjfile = prjfiles[0]

	prj.open(prjfile)

	pkgs = pkgutil.walk_packages(reconos.scripts.__path__, reconos.scripts.__name__ + ".")
	cmds = {}
	for m in [importlib.import_module(m) for f,m,p in pkgs if not p]:
		try:
			if getattr(m, "get_cmd")(None) is not None:
				cmd = getattr(m, "get_cmd")(prj)
				call = getattr(m, "get_call")(prj)
				parser = getattr(m, "get_parser")(prj)
				parser.set_defaults(prj=prj)
				cmds[cmd] = (call, parser)
		except AttributeError:
			log.warning("Could not import Module " + str(m))

	parser = argparse.ArgumentParser(prog="rdk", description="""
	  ReconOS Development Kit - Toolchain integrating different
	  scripts into an extendable development environment for
	  hardware software codesigns.""")
	parser.add_argument("-l", "--log", help="log level", choices=["debug", "info", "warning", "error"], default="warning")
	subparsers = parser.add_subparsers(title="commands", dest="cmd")
	for cmd,attr in cmds.items():
		c = subparsers.add_parser(cmd, add_help=False, parents=[attr[1]])
		c.set_defaults(func=attr[0])

	args = parser.parse_args()

	if args.log == "debug":
		logging.getLogger().setLevel(level=logging.DEBUG)
	elif args.log == "warning":
		logging.getLogger().setLevel(level=logging.WARNING)
	elif args.log == "info":
		logging.getLogger().setLevel(level=logging.INFO)
	else:
		logging.getLogger().setLevel(level=logging.ERROR)

	if args.cmd == None:
		exit = ["exit"]
		def _complete(text, state):
			if " " in readline.get_line_buffer():
				return None

			comp = [_ + " " for _ in cmds.keys() if _.startswith(text)]
			return comp[state] if state < len(comp) else None

		readline.parse_and_bind("tab: complete")
		readline.set_completer(_complete)

		while(True):
			try:
				if args.log == "debug":
					i = input("ReconOS Toolchain [" + prjfile + "]> ")
				else:
					i = input("ReconOS Toolchain> ")
			except KeyboardInterrupt:
				print()
				continue
			except EOFError:
				print()
				sys.exit(0)

			if i in exit:
				sys.exit(0)

			cmd = i.split()[0]
			if cmd in cmds.keys():
				cmd = cmds[cmd]
				try:
					cmd[0](cmd[1].parse_args(i.split()[1:]))
				except SystemExit:
					pass
				except Exception:
					log.error("Executing command '" + i + "' failed")

			else:
				print("Command not found")
	else:
		args.func(args)
