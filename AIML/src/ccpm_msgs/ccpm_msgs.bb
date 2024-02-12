inherit ros_distro_galactic

SUMMARY = "Contains all the CCPM ROS types"
SECTION = "interface"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"
VERSION = "2.1"

SRCREV = "${AUTOREV}"
PV = "${VERSION}.AUTOINC"


ROS_CN = "ccpm_msgs"
SRV_NAME = "${ROS_CN}.service"


ROS_BUILD_DEPENDS = " \
    rosidl-default-generators \
    rosidl-adapter \
    rosidl-adapter-native \
    rosidl-generator-cpp-native \
    rosidl-typesupport-fastrtps-cpp-native \
    rosidl-typesupport-introspection-cpp-native \
    rosidl-typesupport-cpp-native \
    rosidl-typesupport-fastrtps-c-native \
    rosidl-generator-py-native \
    rclcpp \
    rclcpp-action \
    ament-cmake-ros-native \
    ament-cmake-ros \
    ament-cmake \
    ament-cmake-gmock \
    std-msgs \
    action-msgs \
    python3-numpy \
    bash \
    systemd \
"

ROS_BUILDTOOL_DEPENDS = " \
    ament-cmake-auto-native \
"

ROS_EXPORT_DEPENDS = " \
    std-msgs \
    action-msgs \
    systemd \
    bash \
"

ROS_BUILDTOOL_EXPORT_DEPENDS = ""

ROS_EXEC_DEPENDS = " \
    rclcpp \
    rclcpp-action \
    std-msgs \
    action-msgs \
    bash \
    python3-numpy \
    systemd \
    "

# Currently informational only -- see http://www.ros.org/reps/rep-0149.html#dependency-tags.
ROS_TEST_DEPENDS = " \
    ament-lint-auto \
    ament-lint-common \
"

DEPENDS = "${ROS_BUILD_DEPENDS} ${ROS_BUILDTOOL_DEPENDS}"
# Bitbake doesn't support the "export" concept, so build them as if we needed them to build this package (even though we actually
# don't) so that they're guaranteed to have been staged should this package appear in another's DEPENDS.
DEPENDS += "${ROS_EXPORT_DEPENDS} ${ROS_BUILDTOOL_EXPORT_DEPENDS}"
#RDEPENDS:${PN}= " "
RDEPENDS:${PN} += "${ROS_EXEC_DEPENDS}"


SRC_URI = "git://git@gitlab.ceiia.com/ccplink/ccpm-packages/ros-packages/ccpm-msgs.git;protocol=ssh;branch=main"

S = "${WORKDIR}/git"

ROS_BUILD_TYPE = "ament_cmake"

inherit cmake pkgconfig systemd

inherit ros_${ROS_BUILD_TYPE}

DISTRO_FEATURES:append = " systemd"
DISTRO_FEATURES_BACKFILL_CONSIDERED += "sysvinit"
VIRTUAL-RUNTIME_init_manager = "systemd"
VIRTUAL-RUNTIME_initscripts = "systemd-compat-units"


do_install:append() {
    local LIB_DIR="${D}/usr/lib"
    local VERSION="1.0"  # Replace with the appropriate version

    #Install cyclonedd.xml
    install -d ${D}/${sysconfdir}
    install -m 0755 ${S}/network/cyclonedds.xml ${D}/${sysconfdir}

    install -d ${D}/${bindir}
    install -m 0755 ${S}/network/network_config.sh ${D}/${bindir}

    install -d ${D}/${systemd_system_unitdir}/
    install -m 0644 ${S}/network/network_setup.service ${D}/${systemd_system_unitdir}/
    install -d ${D}/${systemd_system_unitdir}/multi-user.target.wants
    ln -sf ../network_setup.service ${D}/${base_libdir}/systemd/system/multi-user.target.wants/

    # Move and rename the non-symbolic .so files
    mv ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_c.so ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_c.so.${VERSION}
    mv ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_introspection_c.so ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_introspection_c.so.${VERSION}
    mv ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_fastrtps_cpp.so ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_fastrtps_cpp.so.${VERSION}
    mv ${LIB_DIR}/lib${ROS_CN}__python.so ${LIB_DIR}/lib${ROS_CN}__python.so.${VERSION}
    mv ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_introspection_cpp.so ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_introspection_cpp.so.${VERSION}
    mv ${LIB_DIR}/lib${ROS_CN}__rosidl_generator_c.so ${LIB_DIR}/lib${ROS_CN}__rosidl_generator_c.so.${VERSION}
    mv ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_fastrtps_c.so ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_fastrtps_c.so.${VERSION}
    mv ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_cpp.so ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_cpp.so.${VERSION}


    # Create symbolic links for the moved and renamed files
    ln -sf lib${ROS_CN}__rosidl_typesupport_c.so.${VERSION} ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_c.so
    ln -sf lib${ROS_CN}__rosidl_typesupport_introspection_c.so.${VERSION} ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_introspection_c.so
    ln -sf lib${ROS_CN}__rosidl_typesupport_fastrtps_cpp.so.${VERSION} ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_fastrtps_cpp.so
    ln -sf lib${ROS_CN}__python.so.${VERSION} ${LIB_DIR}/lib${ROS_CN}__python.so
    ln -sf lib${ROS_CN}__rosidl_typesupport_introspection_cpp.so.${VERSION} ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_introspection_cpp.so
    ln -sf lib${ROS_CN}__rosidl_generator_c.so.${VERSION} ${LIB_DIR}/lib${ROS_CN}__rosidl_generator_c.so
    ln -sf lib${ROS_CN}__rosidl_typesupport_fastrtps_c.so.${VERSION} ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_fastrtps_c.so
    ln -sf lib${ROS_CN}__rosidl_typesupport_cpp.so.${VERSION} ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_cpp.so




}



pkg_postinst_ontarget:${PN} () {
    #!/bin/sh -e
    OPTS=""
    if [ -n "$D" ]; then
        OPTS="--root=$D"
    fi
    systemctl enable network_setup.service


    LIB_DIR="/usr/lib"
    VERSION="1.0"  # Replace with the appropriate version
    ln -sf ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_c.so.${VERSION} ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_c.so
    ln -sf ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_introspection_c.so.${VERSION} ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_introspection_c.so
    ln -sf ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_fastrtps_cpp.so.${VERSION} ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_fastrtps_cpp.so
    ln -sf ${LIB_DIR}/lib${ROS_CN}__python.so.${VERSION} ${LIB_DIR}/lib${ROS_CN}__python.so
    ln -sf ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_introspection_cpp.so.${VERSION} ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_introspection_cpp.so
    ln -sf ${LIB_DIR}/lib${ROS_CN}__rosidl_generator_c.so.${VERSION} ${LIB_DIR}/lib${ROS_CN}__rosidl_generator_c.so
    ln -sf ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_fastrtps_c.so.${VERSION} ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_fastrtps_c.so
    ln -sf ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_cpp.so.${VERSION} ${LIB_DIR}/lib${ROS_CN}__rosidl_typesupport_cpp.so
}


FILES:${PN} = "${PYTHON_SITEPACKAGES_DIR}/${ROS_CN} /usr/share/${ROS_CN} ${includedir}/${ROS_CN} /usr/lib/* ${bindir}/* ${sysconfdir}/*"

