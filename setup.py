from setuptools import setup, find_packages

setup(
    name="BSSD_derivation_for_Lanelet2",
    version="0.9",
    description='Extends Lanelet2 maps with empty BSSD structure and derive behavioral demands',
    author='Jannik Hildebrand',
    author_email='jannik.hildebrand@t-online.de',
    packages=find_packages(),
    install_requires=[
        'numpy==1.22.3',
        'osmium==3.2.0'
    ],
    python_requires='>=3.8'

)
