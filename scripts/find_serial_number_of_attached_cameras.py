import subprocess
serials = {}
FILTER = "ID_SERIAL="


def get_cam_serial(cam_id):
    p = subprocess.Popen('udevadm info --name=/dev/video{} | grep {} | cut -d "=" -f 2'.format(cam_id, FILTER),
                         stdout=subprocess.PIPE, shell=True)
    (output, err) = p.communicate()
    p.status = p.wait()
    response = output.decode('utf-8')
    return response.replace('\n', '')


for cam_id in range(0, 10, 2):
    serial = get_cam_serial(cam_id)
    if len(serial) > 6:
        serials[cam_id] = serial

print('Serial numbers:', serials)