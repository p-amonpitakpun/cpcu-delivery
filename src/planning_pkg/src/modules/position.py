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
        for (_x, _y, direction) in self.data:
            if (x - _x)**2 + (y - _y)**2 < closest:
                closest = (x - _x)**2 + (y - _y)**2
                point = (_x, _y, direction)
        return (closest, point)


class Position:

    def __init__(self):
        self.planned = tuple
        self.line = []
        self.goal = None
        self.error = float('inf')
        self.last_position = None

    def get_position(self, x, y):
        min_len = float('inf')
        _fit = None
        for line in list(filter(lambda line: line.check_point(x, y), self.line)):
            err, point = line.get_position(x, y)
            if err < min_len:
                min_len = err
                _fit = point
        if not _fit:
            return self.last_position
        self.last_position = _fit
        self.error = min_len
        print(x,y,_fit)
        return _fit
    
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