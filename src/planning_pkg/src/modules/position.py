def get_angle_diff(angle1, angle2):
    return (angle1 - angle2 + 180) % 360 - 180

class Line:

    def __init__(self, data, domain, _range):
        self.data = data
        self.domain = range(*domain)
        self.range = range(*_range)

    def check_point(self, x, y):
        if x in self.domain or y in self.range:
            return True
        return False

    def get_position(self, x, y):
        closest = float('inf')
        point = None
        for idx, position in enumerate(self.data):
            if (x - position[0])**2 + (y - position[1])**2 < closest:
                closest = (x - position[0])**2 + (y - position[1])**2
                point = position
                if idx < len(self.data) - 1:
                    next_point = self.data[idx + 1]
                else:
                    next_point = None
        return (closest, point, next_point)


class Position:

    def __init__(self):
        self.planned = tuple
        self.current_line_num = 0
        self.line = []
        self.goal = None
        self.error = float('inf')

    def get_position(self, x, y, direction):
        min_len = float('inf')
        _fit = []
        for line in self.line:
            if not line.check_point(int(x), int(y)):
                continue
            err, point, next_point = line.get_position(x, y)
            if err < min_len:
                min_len = err
                _fit = [(point, next_point)]
            elif err == min_len:
                _fit.append((point, next_point))
        if not _fit:
            return None
        self.error = min_len
        if len(_fit) > 1:
            if abs(get_angle_diff(direction, _fit[0][0][2])) < abs(get_angle_diff(direction, _fit[1][0][2])):
                return _fit[1][0]
            return _fit[1][1]
        return _fit[0][1]
    
    def threshold_check(self, thres):
        return self.error >= thres

    def update_plan(self, plan):
        if not plan:
            return
        self.planned = plan
        self.goal = list(plan)[len(plan) - 1][:2]
        line_data = []
        _x = set()
        _y = set()
        prev_direction = None
        for (x, y, direction) in self.planned:
            if direction != prev_direction and line_data:
                self.line.append(
                    Line(line_data, (min(_x), max(_x) + 1), (min(_y), max(_y) + 1)))
                line_data = []
                _x = set()
                _y = set()
            line_data.append((x, y, direction))
            _x.add(x)
            _y.add(y)
            prev_direction = direction
        self.line.append(Line(line_data, (min(_x), max(_x) + 1), (min(_y), max(_y) + 1)))

    def at_goal(self, current_position):
        return self.goal == current_position