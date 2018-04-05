class Landmark(object):

    def __init__(self, split_line):
        self.name = split_line[0]
        self.x = float(split_line[1])
        self.y = float(split_line[2])
        self.angle = float(split_line[3])

    def print_landmark(self):
        print self.name + ' ' + str(self.x) + ' ' + str(self.y) + ' ' + str(self.angle)
