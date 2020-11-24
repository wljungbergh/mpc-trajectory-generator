import tkinter as tk

class ExampleApp(tk.Tk):
    def __init__(self):
        tk.Tk.__init__(self)
        self.previous_x = self.previous_y = 0
        self.x = self.y = 0
        self.boundary_points = []
        self.drawing_boundary = False
        self.drawing_obstacle = False
        self.polygon_points = []
        self.points_recorded = []
        self.line_fill_color = 'yellow'
        self.canvas = tk.Canvas(self, width=400, height=400, bg = "black", cursor="cross")
        self.line_to_mouse = None
        self.canvas.pack(side="top", fill="both", expand=True)
        self.button_start_boundary = tk.Button(self, text = "Start boundary", command = self.start_boundary)
        self.button_start_boundary.pack(side="top", fill="both", expand=True)
        self.button_finish_boundary = tk.Button(self, text = "Finish boundary", command = self.finish_boundary, state = 'disabled')
        self.button_finish_boundary.pack(side="top", fill="both", expand=True)
        self.button_start_obstacle = tk.Button(self, text = "Start obstacle", command = self.start_obstacle)
        self.button_start_obstacle.pack(side="top", fill="both", expand=True)
        self.button_finish_obstacle = tk.Button(self, text = "Finish obstacle", command = self.finish_obstacle, state = 'disabled')
        self.button_finish_obstacle.pack(side="top", fill="both", expand=True)
        self.clear_button = tk.Button(self, text = "Clear", command = self.clear_all)
        self.clear_button.pack(side="top", fill="both", expand=True)
        self.canvas.bind("<Motion>", self.tell_me_where_you_are)
        self.canvas.bind("<ButtonPress-1>", self.draw_from_where_you_are)

    def clear_all(self):
        self.canvas.delete("all")

    def start_boundary(self):
        self.line_fill_color = 'yellow'
        self.drawing_boundary = True
        self.button_start_boundary["state"] = 'disabled'
        self.button_start_obstacle["state"] = 'disabled'
        self.button_finish_boundary["state"] = 'normal'
        self.points_recorded = []
    
    def finish_boundary(self):
        self.boundary_points = self.points_recorded
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
        self.polygon_points.append(self.points_recorded)
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
        self.previous_x = event.x
        self.previous_y = event.y

    def draw_from_where_you_are(self, event):
        if not self.drawing_obstacle and not self.drawing_boundary:
            return

        self.x = event.x
        self.y = event.y
        if len(self.points_recorded):
            self.canvas.create_line(self.points_recorded[-2], self.points_recorded[-1], 
                                    self.x, self.y,fill=self.line_fill_color)
        self.points_recorded.append(self.x)     
        self.points_recorded.append(self.y)        
        self.previous_x = self.x
        self.previous_y = self.y

if __name__ == "__main__":
    app = ExampleApp()
    app.mainloop()
    print(app.boundary_points)
    print(app.polygon_points)