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
#   description:  A utils script wrapping os and shutil commands.
# 
# ======================================================================

import os
import shutil
import re

def environ(var):
	if var in os.environ:
		return os.environ[var]
	else:
		return None

def setenviron(var, value):
	os.environ[var] = value

def expandenvironvars(string):
	return os.path.expandvars(string)

def chdir(path):
	return os.chdir(path)

def getcwd():
	return os.getcwd()

def rename(src, dst):
	if not os.path.exists(dst):
		os.rename(src, dst)

def join(path, *paths):
	return os.path.join(path, *paths)

def isdir(path):
	return os.path.isdir(path)

def isfile(path):
	return os.path.isfile(path)

def islink(path):
	return os.path.islink(path)

def exists(path):
	return os.path.exists(path)

def basename(path):
	return os.path.basename(path)

def dirname(path):
	return os.path.dirname(os.path.realpath(path))

def trimext(path):
	return os.path.splitext(path)[0]

def getext(path):
	return os.path.splitext(path)[1]

def mkdir(path):
	if not os.path.isdir(path):
		os.mkdir(path)

def listdirs(path, rec=False):
	if rec:
		return [_[0] for _ in os.walk(path)]
	else:
		return [_ for _ in os.listdir(path) if os.path.isdir(os.path.join(path, _))]

def listfiles(path, rec=False, ext=None, rel=True):
	if rec:
		ls = [os.path.join(l[0], _) for l in os.walk(path, followlinks=True) for _ in l[2]]
	else:
		ls = [_ for _ in os.listdir(path) if os.path.isfile(os.path.join(path, _))]
	if ext is not None:
		ls = [_ for _ in ls if re.match(ext, getext(_).lstrip('.'))]

	if rel:
		return [os.path.relpath(_, path) for _ in ls]
	else:
		return ls

def abspath(path):
	return os.path.abspath(path)

def relpath(path, rel):
	return os.path.relpath(path, rel)

def symlink(src, dst, rel=True):
	if rel:
		link = os.path.relpath(src, os.path.join(dst, ".."))
		os.symlink(link, dst)
	else:
		os.symlink(src, dst)


def walk(path, ffunc, dfunc=None, ext=None, followlinks=False):
	for root, dirs, files in os.walk(path):
		for f in files:
			f = os.path.join(root, f)
			if followlinks or not os.path.islink(f):
				if ext is None or getext(f) in ext:
					ffunc(f)

		if dfunc is not None:
			for d in dirs:
				dfunc(os.path.join(root, d), dirs)

def remove(path):
	if os.path.exists(path):
		os.remove(path)

def rmtree(path):
	if os.path.isdir(path):
		shutil.rmtree(path)

def copytree(src, dst, followlinks=False):
	if os.path.isfile(src):
		shutil.copy2(src, dst, follow_symlinks=followlinks)
	else:
		for root, dirs, files in os.walk(src, followlinks=followlinks):
			for d in dirs:
				src_ = os.path.join(root, d)
				dst_ = os.path.join(dst, os.path.relpath(root, src), d)
				if os.path.islink(src_) and not followlinks:
					#print("Copying symlink '" + src_ + "' to '" + dst_ + "'")
					shutil.copy2(src_, dst_, follow_symlinks=False)
				else:
					#print("Making dir '" + dst_ + "'")
					if not os.path.isdir(dst_):
						os.mkdir(dst_)
			for f in files:
				src_ = os.path.join(root, f)
				dst_ = os.path.join(dst, os.path.relpath(root, src), f)
				if os.path.islink(src_) and not followlinks:
					#print("Copying symlink '" + src_ + "' to '" + dst_ + "'")
					shutil.copy2(src_, dst_, follow_symlinks=False)
				else:
					#print("Copying '" + src_ + "' to '" + dst_ + "'")
					shutil.copy2(src_, dst_)

def linktree(src, dst):
	for x in os.listdir(src):
		link = os.path.relpath(os.path.join(src, x), dst)
		os.symlink(link, os.path.join(dst, x))