import tkinter as tk


class ExampleApp(tk.Tk):
    def __init__(self):
        tk.Tk.__init__(self)
        self.x = self.y = 0
        self.boundary_points = []
        self.polygon_points = []
        self.points_recorded = []

        self.drawing_boundary = False
        self.drawing_obstacle = False

        self.line_fill_color = 'yellow'
        self.line_to_mouse = None

        self.canvas = tk.Canvas(
            self, width=600, height=600, bg="black", cursor="cross")
        self.canvas.pack(side="top", fill="both", expand=True)

        self.button_start_boundary = tk.Button(
            self, text="Start boundary", command=self.start_boundary)
        self.button_start_boundary.pack(side="top", fill="both", expand=True)

        self.button_finish_boundary = tk.Button(
            self, text="Finish boundary", command=self.finish_boundary, state='disabled')
        self.button_finish_boundary.pack(side="top", fill="both", expand=True)

        self.button_start_obstacle = tk.Button(
            self, text="Start obstacle", command=self.start_obstacle)
        self.button_start_obstacle.pack(side="top", fill="both", expand=True)

        self.button_finish_obstacle = tk.Button(
            self, text="Finish obstacle", command=self.finish_obstacle, state='disabled')
        self.button_finish_obstacle.pack(side="top", fill="both", expand=True)

        self.button_clear = tk.Button(
            self, text="Clear", command=self.clear_all)
        self.button_clear.pack(side="top", fill="both", expand=True)

        self.button_finish = tk.Button(
            self, text="Finish", command=self.finish)
        self.button_finish.pack(side="top", fill="both", expand=True)

        self.canvas.bind("<Motion>", self.tell_me_where_you_are)
        self.canvas.bind("<ButtonPress-1>", self.draw_from_where_you_are)

    def clear_all(self):
        self.canvas.delete("all")
        self.boundary_points = []
        self.polygon_points = []
        self.drawing_boundary = False
        self.drawing_obstacle = False
        if self.line_to_mouse:
            self.canvas.delete(self.line_to_mouse)
        self.reset_button_states()

    def finish(self):
        self.destroy()

    def reset_button_states(self):
        self.button_start_boundary["state"] = 'normal'
        self.button_start_obstacle["state"] = 'normal'
        self.button_finish_boundary["state"] = 'disable'
        self.button_finish_obstacle["state"] = 'disable'

    def start_boundary(self):
        self.line_fill_color = 'yellow'
        self.drawing_boundary = True
        self.button_start_boundary["state"] = 'disabled'
        self.button_start_obstacle["state"] = 'disabled'
        self.button_finish_boundary["state"] = 'normal'
        self.points_recorded = []

    def finish_boundary(self):
        self.boundary_points = [(x, y) for x, y in zip(
            self.points_recorded[0::2], self.points_recorded[1::2])]
        self.canvas.create_line(self.points_recorded[-2], self.points_recorded[-1],
                                self.points_recorded[0], self.points_recorded[1], fill='yellow')

        self.points_recorded = []
        self.canvas.delete(self.line_to_mouse)
        self.drawing_boundary = False

        self.button_start_boundary["state"] = 'disabled'
        self.button_start_obstacle["state"] = 'normal'
        self.button_finish_boundary["state"] = 'disabled'

    def start_obstacle(self):
        self.line_fill_color = 'green'
        self.button_start_boundary["state"] = 'disabled'
        self.button_start_obstacle["state"] = 'disabled'
        self.button_finish_boundary["state"] = 'disabled'
        self.button_finish_obstacle["state"] = 'normal'
        self.points_recorded = []

        self.drawing_obstacle = True

    def finish_obstacle(self):
        self.polygon_points.append([(x, y) for x, y in zip(
            self.points_recorded[0::2], self.points_recorded[1::2])])
        self.canvas.create_line(self.points_recorded[-2], self.points_recorded[-1],
                                self.points_recorded[0], self.points_recorded[1], fill=self.line_fill_color)

        self.points_recorded = []

        self.canvas.delete(self.line_to_mouse)

        self.button_start_boundary["state"] = 'disabled'
        self.button_start_obstacle["state"] = 'normal'
        self.button_finish_boundary["state"] = 'disabled'
        self.button_finish_obstacle["state"] = 'disabled'

        self.drawing_obstacle = False

    def tell_me_where_you_are(self, event):
        if (self.drawing_obstacle or self.drawing_boundary) and len(self.points_recorded):
            if self.line_to_mouse:
                self.canvas.delete(self.line_to_mouse)
            self.line_to_mouse = self.canvas.create_line(self.points_recorded[-2], self.points_recorded[-1],
                                                         event.x, event.y, fill=self.line_fill_color)

    def draw_from_where_you_are(self, event):
        if not self.drawing_obstacle and not self.drawing_boundary:
            return

        self.x = event.x
        self.y = event.y
        if len(self.points_recorded):
            self.canvas.create_line(self.points_recorded[-2], self.points_recorded[-1],
                                    self.x, self.y, fill=self.line_fill_color)
        self.points_recorded.append(self.x)
        self.points_recorded.append(self.y)


if __name__ == "__main__":
    app = ExampleApp()
    app.mainloop()
    print(f"Boundary coordinates: {app.boundary_points}")
    print(f"Obstacle coordinates: {app.polygon_points}")
