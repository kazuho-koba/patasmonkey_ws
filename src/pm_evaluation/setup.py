from setuptools import find_packages, setup


package_name = "pm_evaluation"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="kazuho",
    maintainer_email="kazuho.kobayashi.ynu@gmail.com",
    description=(
        "Offline evaluation tools for Patasmonkey rosbag data."
    ),
    license="MIT",
    tests_require=[
        "pytest",
    ],
    entry_points={
        "console_scripts": [
            (
                "analyze_bag_frequencies = "
                "pm_evaluation.cli.analyze_bag_frequencies:main"
            ),
        ],
    },
)