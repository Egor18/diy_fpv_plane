import pygame
import time
import serial
import datetime

# Button 5  - OSD ON/OFF
# Button 6  - DIM ON/OFF
# Button 12 - SET HOME

PORT_NAME = 'COM3'
BAUD_RATE = 57600

BLACK = (  0,   0,   0)
WHITE = (255, 255, 255)
RED =   (255,   0,   0)

MIN_THROTTLE = 0
MAX_THROTTLE = 50
MIN_POS      = 0
MAX_POS      = 62

class Channel:
    ELEVATOR = 0
    RUDDER   = 1
    AILERON  = 2
    THROTTLE = 3
    CAMERA   = 4
    OSD      = 5

class OSDCommand:
    OSD_OFF   = 51
    OSD_ON    = 52
    DIM_OFF   = 53
    DIM_ON    = 54
    ERR_INC   = 55
    ERR_DEC   = 56
    SET_HOME  = 57
    LOST_CONN = 58

# 00 xxxxxx - channel
# 01 xxxxxx - confirmation
# 10 yyyyyy - data
# 11 yyyyyy - confirmation
class Package:
    def __init__(self, channel, data):
        if channel < 0 or channel > 63:
            raise Exception('Invalid channel argument')
        if data < 0 or data > 63:
            raise Exception('Invalid data argument')
        self.channel = channel
        self.data = data

    def get_bytes(self):
        return bytes([(0 << 6) | self.channel,
                      (1 << 6) | self.channel,
                      (2 << 6) | self.data,
                      (3 << 6) | self.data])

    def send(self, port):
        port.write(self.get_bytes())

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def draw_text(screen, text, x, y):
    textBitmap = pygame.font.SysFont('calibri', 15).render(text, True, BLACK)
    screen.blit(textBitmap, [x, y])

pygame.init()
pygame.display.set_caption(' ')
screen = pygame.display.set_mode([280, 160])
clock = pygame.time.Clock()
start_time = time.time()
last_sync = 0
port = serial.Serial(PORT_NAME, BAUD_RATE)

if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

axis_x = axis_y = axis_z = axis_throttle = 0.0
aileron = elevator = throttle = rudder = camera = MAX_POS // 2

osd_on = True
dim_on = True

done = False
while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
        elif event.type == pygame.JOYBUTTONDOWN:
            if event.button == 4:
                osd_on = not osd_on
                Package(Channel.OSD, OSDCommand.OSD_ON if osd_on else OSDCommand.OSD_OFF).send(port)
            elif event.button == 5:
                dim_on = not dim_on
                Package(Channel.OSD, OSDCommand.DIM_ON if dim_on else OSDCommand.DIM_OFF).send(port)
            elif event.button == 11:
                Package(Channel.OSD, OSDCommand.SET_HOME).send(port)
        elif event.type == pygame.JOYAXISMOTION:
            if event.axis == 0:
                axis_x = event.value
                new_aileron = int(map_range(axis_x, -1.0, 1.0, MIN_POS, MAX_POS) + 0.5)
                if new_aileron != aileron:
                    aileron = new_aileron
                    Package(Channel.AILERON, aileron).send(port)
            elif event.axis == 1:
                axis_y = -event.value
                new_elevator = int(map_range(axis_y, -1.0, 1.0, MIN_POS, MAX_POS) + 0.5)
                if new_elevator != elevator:
                    elevator = new_elevator
                    Package(Channel.ELEVATOR, elevator).send(port)
            elif event.axis == 2:
                axis_throttle = -event.value
                new_throttle = int(map_range(axis_throttle, -1.0, 1.0, MIN_THROTTLE, MAX_THROTTLE) + 0.5)
                if new_throttle != throttle:
                    throttle = new_throttle
                    Package(Channel.THROTTLE, throttle).send(port)
            elif event.axis == 3:
                axis_z = -event.value
                new_rudder = int(map_range(axis_z, -1.0, 1.0, MIN_POS, MAX_POS) + 0.5)
                if new_rudder != rudder:
                    rudder = new_rudder
                    Package(Channel.RUDDER, rudder).send(port)
        elif event.type == pygame.JOYHATMOTION:
            if event.value == (1, 0) and camera < MAX_POS - 1:
                camera += 10
            elif event.value == (-1, 0) and camera > MIN_POS + 1:
                camera -= 10
            elif event.value == (0, 1) or event.value == (0, -1):
                camera = MAX_POS // 2
            Package(Channel.CAMERA, camera).send(port)

    elapsed_seconds = int(time.time() - start_time)
    if elapsed_seconds - last_sync > 0:
        Package(Channel.THROTTLE, throttle).send(port)
        last_sync = elapsed_seconds

    screen.fill(WHITE)
    draw_text(screen, f'{PORT_NAME} - {BAUD_RATE}', 5, 5)
    draw_text(screen, f'THROTTLE: {throttle * 2}%', 5, 30)
    draw_text(screen, f'AILERON: {aileron}', 5, 50)
    draw_text(screen, f'ELEVATOR: {elevator}', 5, 70)
    draw_text(screen, f'RUDDER: {rudder}', 5, 90)
    draw_text(screen, f'CAMERA: {camera}', 5, 110)
    elapsed_time = str(datetime.timedelta(seconds=elapsed_seconds))
    draw_text(screen, f'TIME: {elapsed_time}', 5, 135)

    offset_x = 150
    offset_y = 5
    width = 121
    height = 121
    x = map_range(aileron, MIN_POS, MAX_POS, 3, width - 4)
    y = map_range(MAX_POS - elevator, MIN_POS, MAX_POS, 3, height - 4)
    pygame.draw.rect(screen, RED, [offset_x + x - 3, offset_y + y - 3, 7, 7])
    pygame.draw.rect(screen, BLACK, [offset_x, offset_y, width, height], 1)
    pygame.draw.line(screen, BLACK, [offset_x + width / 2, offset_y], [offset_x + width / 2, offset_y + height - 1], 1)
    pygame.draw.line(screen, BLACK, [offset_x, offset_y + height / 2], [offset_x + width - 1, offset_y + height / 2], 1)

    offset_x = 150
    offset_y = 135
    width = 121
    height = 9
    z = map_range(MAX_POS - rudder, MIN_POS, MAX_POS, 3, width - 4)
    pygame.draw.rect(screen, RED, [offset_x + z - 3, offset_y + 1, 7, 7])
    pygame.draw.rect(screen, BLACK, [offset_x, offset_y, width, height], 1)
    pygame.draw.line(screen, BLACK, [offset_x + width/2, offset_y], [offset_x + width/2, offset_y + height - 1], 1)

    pygame.display.flip()
    clock.tick(30) #30 FPS

pygame.quit()
