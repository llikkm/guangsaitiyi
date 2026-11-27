import math
import pygame as pg


def field_view(screen,c,small):
    scale = min(840/c["width"],580/c["height"])
    left,base = 24,630

    def px(x,y):
        return round(left+x*scale),round(base-y*scale)

    def area(x,y,width,height,color):
        pg.draw.rect(screen,color,(*px(x,y+height),round(width*scale),round(height*scale)))

    colors = [(236,112,96),(223,231,242)]
    def draw(w,team,paths,target=None):
        area(0,0,c["width"],c["height"],(73,92,92))
        s = c["scatter"]
        area((c["width"]-s)/2,(c["height"]-s)/2,s,s,(90,107,102))
        for i in range(2):
            x = c["width"]-c["goal_depth"] if i else 0
            area(x,0,c["goal_depth"],c["height"],(66,85,106) if i else (101,78,76))
            area(x,(c["height"]-c["goal_width"])/2,c["goal_depth"],c["goal_width"],
                 (71,93,131) if i else (127,85,81))
            inset = c["start_inset"]
            area(c["width"]-inset-c["start"] if i else inset,
                 c["height"]-inset-c["start"] if i else inset,c["start"],c["start"],(100,108,106))
            if len(paths[i])>1: pg.draw.lines(screen,colors[i],False,list(paths[i]),2)
        for shape in getattr(w, "obstacles", ()):
            vertices = [px(*shape.body.local_to_world(v)) for v in shape.get_vertices()]
            pg.draw.polygon(screen, (172, 152, 104), vertices)
        car = w.cars[team]
        for angle,distance in w.scans[team]:
            if distance is not None:
                x,y,yaw = w.scan_poses[team]
                a = angle+yaw
                pg.draw.circle(screen,(96,225,175),px(x+distance*math.cos(a),y+distance*math.sin(a)),2)
        for t,k,b in w.pieces:
            p = px(*b.position)
            pg.draw.circle(screen,(25,30,33),p,round(c["diameter"]/2*scale))
            glyph = small.render("GAEHRCP"[k],True,colors[t])
            screen.blit(glyph,glyph.get_rect(center=p))
        for i,bot in enumerate(w.cars):
            pg.draw.polygon(screen,colors[i],[px(*bot.body.local_to_world(v)) for v in bot.shape.get_vertices()])
            for rail in bot.rails:
                pg.draw.line(screen,colors[i],px(*bot.body.local_to_world(rail.a)),
                             px(*bot.body.local_to_world(rail.b)),max(2,round(2*rail.radius*scale)))
            pg.draw.line(screen,(22,34,35),px(*bot.body.position),
                         px(*bot.body.local_to_world((c["length"]/2,0))),3)
        if target: pg.draw.circle(screen,(254,213,103),px(*target),6,2)
    return px, draw
