from setuptools import setup

package_name = "natsu_ir"

setup(
    name=package_name,
    version="0.1.0",
    # ソースは src/ 直下にフラットに置き、それを natsu_ir パッケージとして入れる
    packages=[package_name],
    package_dir={package_name: "src"},
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RRST-NHK-Project",
    maintainer_email="riko26maeda@gmail.com",
    description="IR (NEC) receiver bridge for Natsurobo2026",
    license="MIT",
    entry_points={
        "console_scripts": [
            # ros2 run natsu_ir ir_node
            "ir_node = natsu_ir.ir_node:main",
            # ros2 run natsu_ir ir_led_policy
            "ir_led_policy = natsu_ir.ir_led_policy:main",
        ],
    },
)
