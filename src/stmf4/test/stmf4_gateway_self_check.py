#!/usr/bin/env python3
"""Production contract for native STM32Cube F411 <-> ROS <-> F103 gateway."""
from pathlib import Path
import re, yaml
ROOT=Path(__file__).resolve().parents[1]
WS=ROOT.parents[1]
F411=WS/'f411_pio'
bridge=(ROOT/'src/stmf4_hmi_bridge.cpp').read_text()
hmi=(yaml.safe_load((ROOT/'config/hmi.yaml').read_text()) or {})['stmf4_hmi_bridge']['ros__parameters']
main=(F411/'src/main.cpp').read_text(); vesc_h=(F411/'src/VescGateway.h').read_text(); vesc_cpp=(F411/'src/VescGateway.cpp').read_text()
neo_h=(F411/'src/Neo3Sensors.h').read_text(); neo_cpp=(F411/'src/Neo3Sensors.cpp').read_text(); board=(F411/'src/BoardSupport.cpp').read_text()
ui_cfg=(F411/'src/Config.h').read_text(); ui_menu=(F411/'src/UiMenu.h').read_text(); ui_shell=(F411/'src/UiShell.h').read_text(); ui_touch=(F411/'src/TouchButtons.h').read_text()
pio=(F411/'platformio.ini').read_text(); linker=(F411/'linker/STM32F411CEUX_APP.ld').read_text(); boot=(F411/'bootloader/src/main.c').read_text()
dfu=(F411/'scripts/dfu_upload_blackpill.sh').read_text(); pre=(F411/'scripts/usb_dfu_upload.py').read_text(); stlink=(F411/'scripts/provision_recovery_stlink.sh').read_text()
auto=(WS/'src/navigation/launch/autonomous.launch.py').read_text(); esc_launch=(WS/'src/esc/launch/esc.launch.py').read_text(); stmf4_launch=(ROOT/'launch/stmf4.launch.py').read_text()
def req(v,m):
    if not v: raise AssertionError(m)
# Physical/native HAL ownership.
for token in ('GPIO_PIN_6 | GPIO_PIN_7','GPIO_AF7_USART1','USART1_IRQn'):
    req(token in board, 'F411 USART1 PB6/PB7 VESC mapping missing: '+token)
for token in ('GPIO_PIN_2 | GPIO_PIN_3','GPIO_AF7_USART2','USART2_IRQn'):
    req(token in board, 'F411 USART2 PA2/PA3 GNSS mapping missing: '+token)
req('GPIO_PIN_8 | GPIO_PIN_9' in board and 'GPIO_AF4_I2C1' in board, 'F411 I2C1 PB8/PB9 mapping missing')
req('gUsb' in main and 'Serial.' not in main, 'native application must use one Cube USB CDC owner, not Arduino Serial')
# HMI parity.
for token in ('UiMenuId::OVERVIEW','UiMenuId::ESC_ROOT','UiMenuId::PERCEPTION_ROOT','UiMenuId::NAVIGATION_ROOT','SUBMENU_VISIBLE_CARDS = 3'):
    req(token in ui_cfg+ui_menu+ui_shell, 'native HMI contract missing: '+token)
for token in ('drawOverviewDomainCard(0, UiMenuId::ESC_ROOT','drawOverviewDomainCard(1, UiMenuId::PERCEPTION_ROOT','drawOverviewDomainCard(2, UiMenuId::NAVIGATION_ROOT','drawCarouselFooter'):
    req(token in ui_shell, 'native HMI renderer missing: '+token)
req('SoftKey::UP' not in ui_cfg+ui_touch+ui_shell and 'SoftKey::DOWN' not in ui_cfg+ui_touch+ui_shell, 'obsolete UP/DOWN softkeys returned')
req('HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)' in ui_touch, 'native touch CS initialization missing')
req('uint32_t HmiDisplay::readId()' in (F411/'src/HmiDisplay.cpp').read_text() and '0xD3U' in (F411/'src/HmiDisplay.cpp').read_text(),
    'native TFT controller read-ID diagnostic missing')
# Safety and recovery.
for token in ('pollSafetyIo()','setSafetyStop(gNeo3.safetyPressed())'):
    req(token in main, 'direct safety path missing: '+token)
for token in ('kCommMotorEstop','sendSafetyStop','safety_stop_active_','if (safety_stop_active_)'):
    req(token in vesc_h+vesc_cpp, 'F411->F103 local E-stop missing: '+token)
body=vesc_cpp[vesc_cpp.find('void VescGateway::recoveryTick'):vesc_cpp.find('void VescGateway::publishStatus')]
req('recoverRuntimeUart' in body and 'NVIC_SystemReset' not in body, 'UART recovery must not reboot complete F411 stack')
for token in ('Board_SetWatchdogCallback','Board_WatchdogStart','Board_WatchdogStop','APP_WATCHDOG_TIMEOUT_MS'):
    req(token in main, 'stoppable application watchdog missing: '+token)
req('TIM11' in board, 'TIM11 watchdog hardware missing')
req('extern "C" void SysTick_Handler()' in board and 'HAL_IncTick();' in board,
    'native F411 must provide a strong SysTick_Handler; weak startup alias deadlocks HAL_Delay')
req('IWDG->' not in main, 'IWDG must not interrupt ROM-DFU transactions')
# Baud contract.
f411_b=re.search(r'kBaud\s*=\s*(115200|1000000)',vesc_h); f103_b=re.search(r'F103_VESC_UART_BAUD\s+(115200|1000000)u',(WS/'hoverboard-vesc/Src/vesc/f103_boot_layout.h').read_text())
req(f411_b and f103_b and f411_b.group(1)==f103_b.group(1),'F411/F103 UART baud mismatch')
req(int(hmi['serial_baud'])==1000000,'NUC<->F411 CDC must remain 1 Mbaud')
req(int(hmi['vesc_uart_baud_expected'])==int(f411_b.group(1)),'ROS internal VESC baud expectation mismatch')
# ROS ownership / TCP routing.
for token in ('/stmf4/vesc/runtime_tx','/stmf4/vesc/maintenance_tx','/stmf4/vesc/rx','/stmf4/vesc/mode','/stmf4/vesc/diagnostic_command','VESC:STATUS','VESC:LINE:'):
    req(token in bridge,'ROS F411 bridge missing '+token)
req('vesc_uart_baud_active_' in bridge and 'payload.find("baud=")' in bridge,'ROS bridge F411 UART baud observability missing')
req('rx_count > 0UL' in bridge,'connected state must require actual F103 RX')
req(str(hmi.get('serial_device','')).lower()=='auto','F411 serial selector must use identity auto-discovery')
req('STMICROELECTRONICS' in bridge and 'F411' in bridge and 'CDC' in bridge,'F411 USB identity guard missing')
# GNSS measurement-time and ownership.
req('stampFromMcuMillis(v[1])' in bridge and 'publishGnssMeasurement(now(),' not in bridge,'GNSS MCU timestamp mapping invalid')
req('const bool pvt_stream_fresh' in neo_cpp and 'if (!pvt_stream_fresh)' in neo_cpp,'GNSS config recovery must follow stream freshness')
req('DeclareLaunchArgument("publish_stm32_gnss"' in stmf4_launch and '"publish_stm32_gnss": LaunchConfiguration("publish_stm32_gnss")' in esc_launch and "'publish_stm32_gnss': PythonExpression" in auto,'GNSS source ownership launch propagation missing')
# Flash layout and transactional update.
req('board_build.ldscript = linker/STM32F411CEUX_APP.ld' in pio,'native app linker not selected')
req('ORIGIN = 0x08008000' in linker and 'LENGTH = 0x58000' in linker,'native app flash region invalid')
for token in ('APP_BASE','MANIFEST_ADDR','MANIFEST_MAGIC','crc32_bytes','jump_system_dfu','application_valid'):
    req(token in boot,'resident recovery bootloader missing '+token)
for token in ('0x08008000','0x08060000','readback verified','0x08000000:leave','verify_runtime_app','ACK:PONG'):
    req(token in dfu,'transactional ROM-DFU uploader missing '+token)
for token in ('_blind_software_dfu','_reset_runtime_usb_device','_try_stlink_force_dfu','dev_id != 0x431','_cdc_to_dfu_with_recovery'):
    req(token in pre,'multi-state DFU recovery missing '+token)
req('upload_command = $PROJECT_DIR/scripts/provision_recovery_stlink.sh' in pio,'ST-Link app-only upload still possible through production env')
for token in ('DEV_ID=0x%03X','flash write_image $BOOT 0x08000000','flash write_image erase $APP 0x08008000','flash write_image erase $MANIFEST 0x08060000','ACK:PONG'):
    req(token in stlink,'full ST-Link provision contract missing '+token)

# Realtime transport must not block ROS RX callbacks or the F411 main loop.
req(('active_baud_{kBaud}' in vesc_h or 'active_baud_{kBaud};' in vesc_h) and
        'VESC:BAUD:' in vesc_cpp and 'requested == 115200U' in vesc_cpp and
        'requested == 1000000U' in vesc_cpp and 'requested == 2000000U' in vesc_cpp and
        'VESC:ERR:BAUD_UNSUPPORTED' in vesc_cpp,
        'F411 bounded diagnostic baud whitelist missing')
board_h = (WS / 'f411_pio/include/BoardSupport.h').read_text(encoding='utf-8')
board_cpp = (WS / 'f411_pio/src/BoardSupport.cpp').read_text(encoding='utf-8')
req('HAL_UART_Transmit(handle_' not in board_cpp,
        'F411 UART TX must not use blocking HAL_UART_Transmit')
for token in ('kTxSize = 4096U', 'availableForWrite() const', 'irqTxComplete()', 'tx_dropped_'):
    req(token in board_h, f'F411 interrupt-driven UART TX ring missing: {token}')
for token in ('HAL_UART_Transmit_IT', 'HAL_UART_TxCpltCallback', 'gVescUart.irqTxComplete()'):
    req(token in board_cpp, f'F411 asynchronous UART TX implementation missing: {token}')
for token in ('queuedForWrite() const', 'discardPendingTx()'):
    req(token in board_h, f'F411 bounded/latest runtime queue control missing: {token}')
for token in ('HAL_UART_AbortTransmit', 'tx_head_ = tx_tail_ = tx_pending_ = 0U'):
    req(token in board_cpp, f'F411 safety TX preemption missing: {token}')
for token in ('kRuntimeMaxQueuedBytes = 128U', 'runtime_queue_drop_',
              'gVescUart.queuedForWrite() > kRuntimeMaxQueuedBytes', 'gVescUart.discardPendingTx()'):
    req(token in vesc_h + vesc_cpp, f'F411 stale runtime queue / E-stop contract missing: {token}')
bridge_send = bridge[bridge.find('bool sendVescBytes'):bridge.find('void publishVescConnected')]
req('sleep_until' not in bridge_send and 'wire_sec' not in bridge_send,
        'ROS->F411 VESC callback must not emulate UART pacing with sleeps')
req('Backpressure belongs at the F411 ring' in bridge_send,
        'ROS bridge must document F411-owned UART backpressure')
req("sendLine(line, source == 'R' ? 0 : 1)" in bridge_send,
        'runtime ROS->F411 VESC write must use zero EAGAIN sleep budget')

print('PASS stmf4_gateway_self_check native production')
print('pins: VESC USART1 PB6/PB7 | GNSS USART2 PA2/PA3 | IST8310 I2C1 PB8/PB9 | USB CDC single owner')
