class Point:
    def __init__(self, position_tuple):
        x, y = position_tuple[0], position_tuple[1]
        self.x = x
        self.y = y

    def __str__(self) -> str:
        return "({}, {})".format(self.x, self.y)