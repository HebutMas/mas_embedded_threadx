# CherryUSB（裁剪版）：仅保留本仓库实际编译的部分
# Device 栈 + CDC ACM 类 + DWC2 OTG (STM32 glue)
# 上游：https://github.com/sakumisu/CherryUSB （Apache-2.0，见 LICENSE）

set(_cu ${CMAKE_CURRENT_LIST_DIR})

list(APPEND cherryusb_incs
    ${_cu}/common
    ${_cu}/core
    ${_cu}/class/cdc
    ${_cu}/class/hub
    ${_cu}/port/dwc2
)

list(APPEND cherryusb_srcs
    ${_cu}/core/usbd_core.c
    ${_cu}/class/cdc/usbd_cdc_acm.c
)

if(CONFIG_CHERRYUSB_DEVICE_DWC2_ST)
    list(APPEND cherryusb_srcs
        ${_cu}/port/dwc2/usb_dc_dwc2.c
        ${_cu}/port/dwc2/usb_glue_st.c
    )
endif()
