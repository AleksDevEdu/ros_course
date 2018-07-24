INCLUDEPATH+=/opt/ros/kinetic/include
INCLUDEPATH+=./devel/include
INCLUDEPATH+=/opt/ros/kinetic/include/opencv-3.2.0-dev

SOURCES += \
    src/haha_package/src/talker.cpp \
    src/haha_package/src/listener.cpp \
    src/haha_package/src/vectorsample.cpp \
    src/bodryi_packetik/src/net_sender.cpp \
    src/bodryi_packetik/src/camera_sender.cpp \
    src/bodryi_packetik/src/camera_viewer.cpp

DISTFILES += \
    src/haha_package/CMakeLists.txt \
    src/haha_package/msg/Haha.msg \
    src/haha_package/yahoo.launch \
    src/haha_package/msg/TMsg.msg \
    src/bodryi_packetik/msg/Bodryi.msg \
    src/bodryi_packetik/CMakeLists.txt \
    src/bodryi_packetik/msg/CustomImage.msg \
    src/bodryi_packetik/srv/camera_switch.srv

HEADERS += \
    src/haha_package/src/vectorsample.h \
    devel/include/haha_package/Haha.h \
    devel/include/haha_package/TMsg.h
