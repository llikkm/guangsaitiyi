class Match:
    def __init__(self, c):
        self.c = c
        self.round = 0
        self.tick = 0
        self.scores = [0, 0]
        self.penalty = [0, 0]
        self.result = ""

    @property
    def duration(self):
        return self.c["overtime" if self.round else "duration"]

    def penalize(self, team, points=5):
        if not self.result:
            self.penalty[team] += points
            self.scores[team] -= points

    def update(self, pieces, settled=True):
        if self.result:
            return
        c = self.c
        r = c["diameter"]/2
        self.scores = [-v for v in self.penalty]
        for team, kind, x, y in pieces:
            depth = x if team == 0 else c["width"] - x
            if r < depth < c["goal_depth"]-r and abs(y-c["height"]/2)+r < c["goal_width"]/2:
                self.scores[team] += c["points"][kind]
        self.tick = min(self.tick+1, round(self.duration*c["rate"]))
        timed_out = self.tick >= round(self.duration*c["rate"])
        if self.round:
            hits = [s+p > 0 for s,p in zip(self.scores,self.penalty)]
            if hits[0] != hits[1]:
                self.result = "AB"[hits.index(True)] + " wins"
            elif timed_out or all(hits):
                if c["mass"] != c["mass_b"]:
                    self.result = "A wins" if c["mass"] < c["mass_b"] else "B wins"
                else:
                    self.new_round()
        elif timed_out and settled:
            a,b = self.scores
            if a == b:
                self.new_round()
            else:
                self.result = "A wins" if a>b else "B wins"

    def new_round(self):
        self.round += 1
        self.tick = 0
        self.scores = [0,0]
        self.penalty = [0,0]
