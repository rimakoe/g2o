from setuptools import setup
from setuptools.command.build import build
import shutil
import os
import glob
import subprocess
from wheel.bdist_wheel import bdist_wheel

class CustomBdistWheel(bdist_wheel):
    def run(self):
        print("Running custom bdist_wheel command...")
        # You can add pre-processing logic here before the wheel is built
        super().run()  # Calls the original bdist_wheel command
        print("Finished building the wheel!")

class CustomBuildCommand(build):
    def run(self):
        build_filepaths = glob.glob(os.path.join(os.getcwd(), "build", "lib", "g2opy*.so"))  # Linux/macOS
        if build_filepaths:
            build_filepath= build_filepaths[0]
        else:
            print("No g2opy shared lib found!")
        dest_filepath = os.path.join(os.getcwd(), "build", "python", "lib","g2opy", os.path.basename(build_filepath))
        os.makedirs(os.path.dirname(dest_filepath), exist_ok=True)
        shutil.copy(build_filepath, dest_filepath)
        print(f"Copied G2O binding: {build_filepath} -> {dest_filepath}")

        with open(os.path.join(os.getcwd(), "build", "python", "lib","g2opy", "__init__.py"), "w") as f:
            f.write("from .g2opy import *\n")
        
        with open(os.path.join(os.getcwd(), "build", "python", "lib","g2opy", "py.typed"), "w") as f:
            f.write("\n")

        stub_output = os.path.join(os.getcwd(), "build", "python", "lib","g2opy")
        os.makedirs(stub_output, exist_ok=True)
        subprocess.run(["stubgen","-m", "g2opy", "-o", "."], cwd=stub_output, check=True)
        print(f"Generated type hints in: {stub_output}")
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
        cmdclass = {'build': CustomBuildCommand, 'bdist_wheel': CustomBdistWheel},
    )
