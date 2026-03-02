from setuptools import find_packages, setup

package_name = "ailinker"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "requests", "websockets"],
    zip_safe=True,
    maintainer="shaon",
    maintainer_email="hshaon@gmail.com",
    description="AILinker tool server + Gemini orchestrator for ATRBridge",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ailinker_server = ailinker.ailinker_server:main",
            "ai_behavior_orchestrator = ailinker.llm_orchestrator:main",
        ],
    },
)
