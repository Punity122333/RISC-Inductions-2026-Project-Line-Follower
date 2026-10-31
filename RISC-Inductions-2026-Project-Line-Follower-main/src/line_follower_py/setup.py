import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'line_follower_py'


def files(*patterns):
    """Glob several patterns, keep regular files only (dirs break data_files)."""
    out = []
    for p in patterns:
        out += [f for f in glob(p, recursive=True) if os.path.isfile(f)]
    return sorted(set(out))


def install_tree(src_dir):
    """Install a whole directory tree, preserving subfolders, under share/<pkg>/."""
    entries = []
    for root, _, _ in os.walk(src_dir):
        fs = [os.path.join(root, f) for f in os.listdir(root)
              if os.path.isfile(os.path.join(root, f))]
        if fs:
            entries.append((os.path.join('share', package_name, root), fs))
    return entries


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         files('launch/*.py', 'launch/*.rviz', 'launch/*.yaml')),
        (os.path.join('share', package_name, 'urdf'),
         files('urdf/*.urdf', 'urdf/*.xacro', 'urdf/*.stl')),
        (os.path.join('share', package_name, 'worlds'),
         files('worlds/*.sdf', 'worlds/*.world')),
        (os.path.join('share', package_name, 'meshes'),
         files('meshes/*')),
    ]
    # Gazebo models / textures (keeps subfolders); harmless if folder is absent
    + (install_tree('models') if os.path.isdir('models') else [])
    + (install_tree('config') if os.path.isdir('config') else []),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parth',
    maintainer_email='parth@todo.todo',
    description='ROS 2 camera-based line follower simulation and PD controller',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'line_detector = line_follower_py.line_detector:main',
            'gazebo_line_sim = line_follower_py.gazebo_line_sim:main',
            'track_publisher = line_follower_py.track_publisher:main',
        ]
    },
)