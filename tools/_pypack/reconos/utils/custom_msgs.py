import reconos.utils.shutil2 as shutil2

import logging


log = logging.getLogger(__name__)


"""
Get all names of custom message packages
"""
def get_packages(project_dir):
    msg_directory = shutil2.join(project_dir, "msg")
    if shutil2.exists(msg_directory):
        return shutil2.listdirs(msg_directory)
    return []

"""
Get all absolute paths to the include directories of custom message packages
"""
def get_absolute_include_paths(project_dir):
    packages = get_packages(project_dir)
    install_directory = shutil2.join(project_dir, "build.msg", "install")
    if shutil2.exists(install_directory):
        return [shutil2.join(install_directory, package, "include") for package in packages]
    else:
        if len(packages) > 0:
            log.error("Missing installation directory for messages. Run 'rdk export_msg' and 'rdk build_msg' first.")
        return []
