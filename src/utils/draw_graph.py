import tkinter as tk


class ExampleApp(tk.Tk):
    def __init__(self, scale = 10):
        tk.Tk.__init__(self)
        self.x = self.y = 0
        self.scale = scale
        self.boundary_points = []
        self.polygon_points = []
        self.start_position = []
        self.end_position = []
        self.latest_lines = []
        self.start_position_marker = None
        self.end_position_marker = None
        self.points_recorded = []
        self.mouse_coord_text = None

        self.drawing_boundary = False
        self.drawing_obstacle = False
        self.setting_start = False
        self.setting_end = False

        self.line_fill_color = 'yellow'
        self.line_to_mouse = None

        self.protocol("WM_DELETE_WINDOW", self.close_window)

        self.canvas = tk.Canvas(
            self, width=600, height=600, bg="black", cursor="cross")
        self.canvas.grid(column=0, row=0, columnspan=2)

        self.button_start_boundary = tk.Button(
            self, text="Start boundary", command=self.start_boundary)
        self.button_start_boundary.grid(column=0, row=1, columnspan=1, sticky=tk.N+tk.S+tk.E+tk.W)

        self.button_finish_boundary = tk.Button(
            self, text="Finish boundary", command=self.finish_boundary, state='disabled')
        self.button_finish_boundary.grid(column=1, row=1, columnspan=1, sticky=tk.N+tk.S+tk.E+tk.W)

        self.button_start_obstacle = tk.Button(
            self, text="Start obstacle", command=self.start_obstacle)
        self.button_start_obstacle.grid(column=0, row=2, columnspan=1, sticky=tk.N+tk.S+tk.E+tk.W)

        self.button_finish_obstacle = tk.Button(
            self, text="Finish obstacle", command=self.finish_obstacle, state='disabled')
        self.button_finish_obstacle.grid(column=1, row=2, columnspan=1, sticky=tk.N+tk.S+tk.E+tk.W)

        self.button_clear = tk.Button(
            self, text="Clear", command=self.clear_all)
        self.button_clear.grid(column=0, row=3, columnspan=1, sticky=tk.N+tk.S+tk.E+tk.W)

        self.button_finish = tk.Button(
            self, text="Finish", command=self.finish)
        self.button_finish.grid(column=1, row=3, columnspan=1, sticky=tk.N+tk.S+tk.E+tk.W)

        self.entry_scale = tk.Entry(self)
        self.entry_scale.insert(0,str(self.scale))
        self.entry_scale.grid(column=0, row=4, columnspan=1, sticky=tk.N+tk.S+tk.E+tk.W)

        self.button_update_scale = tk.Button(self, text='Update pixel/meter', command=self.update_scale)
        self.button_update_scale.grid(column=1, row=4, columnspan=1, sticky=tk.N+tk.S+tk.E+tk.W)

        self.button_set_start = tk.Button(
            self, text="Set starting position", command=self.set_start_position)
        self.button_set_start.grid(column=0, row=5, columnspan=1, sticky=tk.N+tk.S+tk.E+tk.W)

        self.button_set_end = tk.Button(
            self, text="Set ending position", command=self.set_end_position)
        self.button_set_end.grid(column=1, row=5, columnspan=1, sticky=tk.N+tk.S+tk.E+tk.W)

        self.button_abort_current = tk.Button(
            self, text="Abort current operation", command=self.abort_current)
        self.button_abort_current.grid(column=0, row=6, columnspan=2, sticky=tk.N+tk.S+tk.E+tk.W)
        self.button_abort_current["state"] = "disable"

        self.canvas.bind("<Motion>", self.tell_me_where_you_are)
        self.canvas.bind("<ButtonPress-1>", self.draw_from_where_you_are)

    def clear_all(self):
        self.canvas.delete("all")
        self.boundary_points = []
        self.polygon_points = []
        self.start_position = []
        self.end_position = []
        self.latest_lines = []
        self.points_recorded = []
        self.drawing_boundary = False
        self.drawing_obstacle = False
        self.setting_start = False
        self.setting_end = False
        if self.line_to_mouse:
            self.canvas.delete(self.line_to_mouse)
        self.reset_button_states()

    def finish(self):
        self.close_window()
    def is_clock_wise(self, poly):
        # based on https://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order/1180256#1180256
        sum = 0
        for i, point in enumerate(poly):
            if i < len(poly)-1:
                sum += (poly[i+1][0]-point[0])*(poly[i+1][1]+point[1])
            else:
                sum += (poly[0][0]-point[0])*(poly[0][1]+point[1])
        return sum >= 0

    def close_window(self):
        scaled_boundary = [(x/self.scale, (600-y)/self.scale) for (x,y) in self.boundary_points]
        if not self.is_clock_wise(scaled_boundary):
            scaled_boundary = scaled_boundary.reverse()

        print(f"Boundary coordinates: {scaled_boundary}")

        scaled_polygons = []
        for poly in self.polygon_points:
            poly = [(x/self.scale, (600-y)/self.scale) for (x,y) in poly]
            if self.is_clock_wise(poly):
                scaled_polygons.append(poly.reverse())
            else:
                scaled_polygons.append(poly)
        print(f"Obstacle coordinates: {scaled_polygons}")

        scaled_start = [(x/self.scale, (600-y)/self.scale) for (x,y) in self.start_position]
        print(f"Starting cooridnates: {scaled_start}")

        scaled_end = [(x/self.scale, (600-y)/self.scale) for (x,y) in self.end_position]
        print(f"Ending cooridnates: {scaled_end}")
        self.destroy()


    def update_scale(self):
        self.scale = float(self.entry_scale.get())

    def reset_button_states(self):
        if len(self.boundary_points):
            self.button_start_boundary["state"] = 'disable'
        else:
            self.button_start_boundary["state"] = 'normal'

        self.button_start_obstacle["state"] = 'normal'
        self.button_finish_boundary["state"] = 'disable'
        self.button_finish_obstacle["state"] = 'disable'
        self.button_abort_current["state"] = "disable"
        self.button_set_start["state"] = "normal"
        self.button_set_end["state"] = "normal"

    def set_start_position(self):
        self.setting_start = True
        self.button_start_boundary["state"] = 'disabled'
        self.button_start_obstacle["state"] = 'disabled'
        self.button_set_start["state"] = 'disabled'
        self.button_set_end["state"] = 'disabled'
        self.button_finish_boundary["state"] = 'disabled'
        self.button_finish_obstacle["state"] = 'disable'
        self.button_abort_current["state"] = "normal"

    def save_start_position(self, x, y):
        self.setting_start = False
        self.start_position = [(x, y)]
        if self.start_position_marker:
            self.canvas.delete(self.start_position_marker)

        self.start_position_marker = self.canvas.create_rectangle(x-3,y-3,x+3,y+3, fill='green')
        self.reset_button_states()

    def set_end_position(self):
        self.setting_end = True
        self.button_start_boundary["state"] = 'disabled'
        self.button_start_obstacle["state"] = 'disabled'
        self.button_set_start["state"] = 'disabled'
        self.button_set_end["state"] = 'disabled'
        self.button_finish_boundary["state"] = 'disabled'
        self.button_finish_obstacle["state"] = 'disable'
        self.button_abort_current["state"] = "normal"

    def save_end_position(self, x, y):
        self.setting_end = False
        self.end_position = [(x, y)]
        if self.end_position_marker:
            self.canvas.delete(self.end_position_marker)

        self.end_position_marker = self.canvas.create_rectangle(x-3,y-3,x+3,y+3, fill='red')
        self.reset_button_states()

    def start_boundary(self):
        self.line_fill_color = 'yellow'
        self.drawing_boundary = True
        self.button_start_boundary["state"] = 'disabled'
        self.button_start_obstacle["state"] = 'disabled'
        self.button_set_start["state"] = 'disabled'
        self.button_set_end["state"] = 'disabled'
        self.button_finish_boundary["state"] = 'normal'
        self.button_abort_current["state"] = "normal"
        self.points_recorded = []
        self.latest_lines = []

    def finish_boundary(self):
        self.boundary_points = [(x, y) for x, y in zip(
            self.points_recorded[0::2], self.points_recorded[1::2])]
        self.canvas.create_line(self.points_recorded[-2], self.points_recorded[-1],
                                self.points_recorded[0], self.points_recorded[1], fill='yellow')

        self.points_recorded = []
        self.latest_lines = []
        self.canvas.delete(self.line_to_mouse)
        self.drawing_boundary = False

        self.button_start_boundary["state"] = 'disabled'
        self.button_start_obstacle["state"] = 'normal'
        self.button_finish_boundary["state"] = 'disabled'
        self.button_set_start["state"] = 'normal'
        self.button_set_end["state"] = 'normal'
        self.button_abort_current["state"] = "disable"

    def start_obstacle(self):
        self.line_fill_color = 'green'
        self.button_start_boundary["state"] = 'disabled'
        self.button_start_obstacle["state"] = 'disabled'
        self.button_finish_boundary["state"] = 'disabled'
        self.button_set_start["state"] = 'disabled'
        self.button_set_end["state"] = 'disabled'
        self.button_finish_obstacle["state"] = 'normal'
        self.button_abort_current["state"] = "normal"
        self.points_recorded = []
        self.latest_lines = []

        self.drawing_obstacle = True

    def finish_obstacle(self):
        self.polygon_points.append([(x, y) for x, y in zip(
            self.points_recorded[0::2], self.points_recorded[1::2])])
        self.canvas.create_line(self.points_recorded[-2], self.points_recorded[-1],
                                self.points_recorded[0], self.points_recorded[1], fill=self.line_fill_color)

        self.points_recorded = []
        self.latest_lines = []

        self.canvas.delete(self.line_to_mouse)

        if len(self.boundary_points):
            self.button_start_boundary["state"] = 'disabled'
        else:
            self.button_start_boundary["state"] = 'normal'

        self.button_start_obstacle["state"] = 'normal'
        self.button_finish_boundary["state"] = 'disabled'
        self.button_finish_obstacle["state"] = 'disabled'
        self.button_set_start["state"] = 'normal'
        self.button_set_end["state"] = 'normal'
        self.button_abort_current["state"] = "disable"

        self.drawing_obstacle = False

    def abort_current(self):
        self.drawing_boundary = False
        self.drawing_obstacle = False
        self.setting_start = False
        self.setting_end = False

        if self.line_to_mouse:
                self.canvas.delete(self.line_to_mouse)

        self.points_recorded = []
        self.reset_button_states()
        for line in self.latest_lines:
            self.canvas.delete(line)
        
        self.latest_lines = []


    def tell_me_where_you_are(self, event):
        if (self.drawing_obstacle or self.drawing_boundary) and len(self.points_recorded):
            if self.line_to_mouse:
                self.canvas.delete(self.line_to_mouse)
            self.line_to_mouse = self.canvas.create_line(self.points_recorded[-2], self.points_recorded[-1],
                                                         event.x, event.y, fill=self.line_fill_color)

        mouse_coordinates= "{:.2f}, {:.2f}".format(event.x/self.scale, (600-event.y)/self.scale)
        if self.mouse_coord_text:
            self.canvas.delete(self.mouse_coord_text)

        x_loc = event.x+50 if event.x < 500 else event.x-50
        y_loc = event.y + 10 if event.y < 15 else event.y
        y_loc = event.y-10 if event.y > 590 else y_loc
        self.mouse_coord_text = self.canvas.create_text(x_loc, y_loc, fill="white", text = mouse_coordinates)

    def draw_from_where_you_are(self, event):
        if not self.drawing_obstacle and not self.drawing_boundary and not self.setting_start and not self.setting_end:
            return

        if self.setting_start:
            self.save_start_position(event.x, event.y)
            return

        if self.setting_end:
            self.save_end_position(event.x, event.y)
            return

        self.x = event.x
        self.y = event.y
        if len(self.points_recorded):
            line = self.canvas.create_line(self.points_recorded[-2], self.points_recorded[-1],
                                    self.x, self.y, fill=self.line_fill_color)
            self.latest_lines.append(line)
        self.points_recorded.append(self.x)
        self.points_recorded.append(self.y)


if __name__ == "__main__":
    app = ExampleApp()
    app.mainloop()
