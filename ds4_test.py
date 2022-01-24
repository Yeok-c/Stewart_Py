from pyPS4Controller.controller import Controller


class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)


controller = MyController(interface="USB\VID_045E&PID_028E\01", connecting_using_ds4drv=False)
controller.listen()