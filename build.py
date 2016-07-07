#!python3

import glob
import shutil
import os
import sys
import re
import subprocess
import regobj
import functools # for reduce
import itertools
import imp, importlib
import pathlib
import pprint
import pywin
import inspect
import types
import abc
import collections

import abc

def is_container(x):
	return isinstance(x, collections.abc.Container)

def is_sequence(x):
	return isinstance(x, collections.abc.Sequence)

def is_iterable(x):
	return isinstance(x, collections.abc.Iterable)

def is_iterator(x):
	return isinstance(x, collections.abc.Iterator)

def is_generator(x):
	return isinstance(x, collections.abc.Generator)

def is_mapping(x):
	return isinstance(x, collections.abc.Mapping)

def flatten(x):
	if (not isinstance(x, str)) and is_iterable(x):
		ret = []
		for y in x:
			ret = ret + flatten(y)
		return ret
	return [x]

class arduino_builder(object):
	def __init__(self, sketch, fqbn, debug=False):
		self._debug = debug
		self._sketch = sketch

		# locate the following directories, by hook or by crook...
		self.arduino_install_dir = None
		self.arduino_sketch_dir = None
		self.arduino_local_packages_dir = None

		(self.vendor, self.arch, self.board, optstr) = fqbn.split(":")
		opts = [str.split("=") for str in optstr.split(",")]
		self.settings = {optname: optval for (optname, optval) in opts}

		# search in known places in registry where the path to the teensyduino install might be lurking
		try:
			self.arduino_install_dir = filter(lambda x: x.name == "Install_Dir", list(regobj.HKLM.get_subkey("Software").get_subkey("Wow6432Node").get_subkey("Arduino").values()))[0].data
		except Exception as ex:
			if self._debug:
				sys.stderr.print("Warning: Can't get arduino install dir from registry: {0}".format(ex))
			pass
		
		if self.arduino_install_dir is None:
			check_path = os.path.join(os.environ['USERPROFILE'],"Program Files","arduino-1.6.7")
			if os.path.exists(check_path) and os.path.exists(os.path.join(check_path, "arduino-builder.exe")):
				self.arduino_install_dir = check_path

		check_path = os.path.join(os.environ['LOCALAPPDATA'], "Arduino15", "packages")
		if os.path.isdir(check_path):
			self.arduino_local_packages_dir = check_path

		check_path = os.path.join(os.environ['USERPROFILE'], "Documents", "Arduino")
		if os.path.isdir(check_path):
			self.arduino_sketch_dir = check_path

		# once the key directories are found, add the default paths used to construct an arduino-builder command
		if not (self.arduino_install_dir is None):
			self.arduino_builder_exe = os.path.join(self.arduino_install_dir, "arduino-builder.exe")
			
			self.hardware_paths = [
				os.path.join(self.arduino_install_dir, "hardware"),
				self.arduino_local_packages_dir
			]
			self.tools_paths = [
				os.path.join(self.arduino_install_dir, "tools-builder"),
				os.path.join(self.arduino_install_dir, "hardware", "tools", "avr"),
				self.arduino_local_packages_dir
			]
			self.built_in_libraries = [
				os.path.join(self.arduino_install_dir, "libraries")
			]
			self.user_libraries = [
				os.path.join(self.arduino_sketch_dir, "libraries")
			]

		# init the prefs cache
		self.prefs_cache = {}
		self.prefs_override = {}

	def get_extra_opts(self):
		return ["-ide-version=10607", "-warnings=none", "-prefs=build.warn_data_percentage=75", "-verbose"]

	def get_fqbn(self):
		return ":".join([self.vendor, self.arch, self.board, ",".join(["{0}={1}".format(sk,sv) for sk,sv in self.settings.items()])])

	def build(self, opts = None):
		if opts is None:
			opts = []
		assert(not (self.arduino_install_dir is None))
		command_opt = []
		#command_opt = command_opt + ["-compile"]

		p = subprocess.Popen( \
			[self.arduino_builder_exe] + command_opt
			+ ["-logger=machine"]
			+ list(flatten([["-hardware", dir] for dir in self.hardware_paths]))
			+ list(flatten([["-tools", dir] for dir in self.tools_paths]))
			+ list(flatten([["-built-in-libraries", dir] for dir in self.built_in_libraries]))
			+ list(flatten([["-libraries", dir] for dir in self.user_libraries]))
			+ ["-fqbn=" + self.get_fqbn()] #teensy:avr:teensy31:usb=serial,speed=96opt,keys=en-us",
			+ self.get_extra_opts()
			+ flatten(opts) + [self._sketch], 
			stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)

		msgs = []
		while p.returncode is None:
			obuf, ebuf = p.communicate()
			olines = [line.rstrip(b"\r").decode("utf-8") for line in obuf.split(b"\n")]
			elines = [line.rstrip(b"\r").decode("utf-8") for line in ebuf.split(b"\n")]
			msg = olines + elines;
			msg = [line.replace(os.path.join(self.prefs_cache['build.path'], "sketch"), self.prefs_cache['build.source.path']) for line in msg]
			msgs = msgs + msg
			for line in msg:
				print(line, end='\n', file=sys.stdout, flush=True)
		#print((b"\n".join(msgs)).decode("utf-8"))

	def dump_prefs(self, opts=[]):
		command_line = [self.arduino_builder_exe, "-dump-prefs", "-logger=machine"] + \
			list(flatten([["-hardware", dir] for dir in self.hardware_paths])) + \
			list(flatten([["-tools", dir] for dir in self.tools_paths])) + \
			list(flatten([["-built-in-libraries", dir] for dir in self.built_in_libraries])) + \
			list(flatten([["-libraries", dir] for dir in self.user_libraries])) + \
			["-fqbn=" + self.get_fqbn()] + \
			self.get_extra_opts() + \
			flatten(opts) + [self._sketch];
		p = subprocess.Popen(command_line, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=1)

		dumped_prefs = {}
		msgs = []
		while p.returncode is None:
			obuf, ebuf = p.communicate()
			olines = [line.rstrip(b"\r").decode("utf-8") for line in obuf.split(b"\n")]
			elines = [line.rstrip(b"\r").decode("utf-8") for line in ebuf.split(b"\n")]
			for line in olines:
				match = re.search("^([\w\.\-\+\_]*)=(.*)$", line)
				if not (match is None):
					dumped_prefs[match.group(1)]=match.group(2)
				else:
					print(line, end='\n', file=sys.stdout, flush=True)
			for line in elines:
				print(line, end='\n', file=sys.stdout, flush=True)
		self.prefs_cache = dumped_prefs
		#print("\nDone!\n")

import pprint, inspect
import argparse, sys, os
if __name__ == "__main__":
	'''parser = argparse.ArgumentParser()
	parser.add_argument('files', default=sys.stdin, metavar='sketch', nargs='*', type=argparse.FileType('r'),
		help='the .ino file for the sketch to build')
	#parser.add_argument(
	#    '--log', default=sys.stdout, type=argparse.FileType('w'),
	#    help='the file where the sum should be written')
	args = parser.parse_args()
	
	# args.log.close()
	'''
	args = sys.argv
	for i, f in enumerate(args[1:]):
		print(">>> Processing file {0} of {1}: {2}".format(i+1, len(args), f), end='\n', file=sys.stderr, flush=True)
		builder = arduino_builder(sketch=f, fqbn="teensy:avr:teensy31:usb=serial,speed=96opt,keys=en-us")
		builder.dump_prefs()
		pprint.pprint(builder.prefs_cache, stream=sys.stdout, indent=1, width=80, depth=None)
		pprint.pprint(inspect.getmembers(builder))
		builder.build()