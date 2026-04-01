from glob import glob
import os

from setuptools import find_packages, setup


package_name = "abb_pi0_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (
            f"share/{package_name}",
            [
                "package.xml",
                "README.md",
                "ROBOTSTUDIO_PI0_SETUP.md",
                "PI0_POLICY_HTTP_PROTOCOL.md",
            ],
        ),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Heng Wu",
    maintainer_email="hang-wu23@mails.tsinghua.edu.cn",
    description="Phase-1 mock pi0 bridge for ABB ROS2 deployments.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "abb_pi0_bridge_node = abb_pi0_bridge.bridge_node:main",
            "pi0_policy_stub_server = abb_pi0_bridge.policy_stub_server:main",
            "robotstudio_preflight = abb_pi0_bridge.robotstudio_preflight:main",
        ],
    },
)
