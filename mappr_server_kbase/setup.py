from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    name='mappr_server_kbase',
    version='0.0.1',
    description='mappr_server_kbase builds on top of kbase to save world information into a mongo db.',
    url='---none---',
    author='David Leins',
    author_email='dleins@techfak.uni-bielefeld.de',
    license='Apache 2.0',
    packages=['mappr_server_kbase'],
    package_dir={'': 'src'}
)

setup(**setup_args)
