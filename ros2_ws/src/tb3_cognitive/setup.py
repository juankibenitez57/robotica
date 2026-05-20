import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tb3_cognitive'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juanks',
    maintainer_email='juankibenitez57@gmail.com',
    description='Capa cognitiva TB3',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    # Wrapper con shebang explícito #!/opt/ai-venv/bin/python3
    # setup.cfg redirige la instalación a lib/tb3_cognitive/
    scripts=[
        'scripts/nlp_intent_node',
        'scripts/cognitive_agent_node',
        'scripts/fsm_node',
    ],
)
