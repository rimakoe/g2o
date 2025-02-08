from setuptools import setup
from setuptools.command.build import build
import shutil
import os
import glob
import subprocess
from wheel.bdist_wheel import bdist_wheel

BUILD_DIR = os.path.join(os.getcwd(), "build", "lib")
PACKAGE_DIR = os.path.join(os.getcwd(), "build", "python", "lib","g2opy")

class CustomBuildCommand(build):
    def run(self):
        build_filepaths = glob.glob(os.path.join(BUILD_DIR, "g2opy*.so"))  # Find the python binding .so file
        if build_filepaths:
            build_filepath= build_filepaths[0]
        else:
            print("No g2o python bindings found in " + BUILD_DIR)
            return
        print("Found python bindings: " + build_filepath)
        package_filepath = os.path.join(PACKAGE_DIR, os.path.basename(build_filepath))
        os.makedirs(PACKAGE_DIR, exist_ok=True)
        shutil.copy(build_filepath, package_filepath)
        print(f"Copied g2o python bindings: {build_filepath} -> {package_filepath}")
        with open(os.path.join(PACKAGE_DIR, "__init__.py"), "w") as f:
            f.write("from .g2opy import *\n") # Create this for no double import in the code        
        with open(os.path.join(PACKAGE_DIR, "py.typed"), "w") as f:
            f.write("\n") # Create this for mypy to find the .pyi 
        subprocess.run(["stubgen","-m", "g2opy", "-o", "."], cwd=PACKAGE_DIR, check=True)
        print(f"Generated type hints in: {PACKAGE_DIR}")
        super().run()

if __name__=="__main__":
    setup(
        name="g2opy",
        version="1.0.0",
        packages=["g2opy"],
        package_dir={"g2opy": "build/python/lib"},
        package_data={"g2opy": ["*"]},
        include_package_data=True,
        install_requires=[
            "mypy",
        ],
        cmdclass = {'build': CustomBuildCommand},
    )
