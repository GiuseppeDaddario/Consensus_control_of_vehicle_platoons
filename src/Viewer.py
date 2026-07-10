import pygame
import numpy as np
from itertools import cycle


class Viewer:
    def __init__(self, width=1600, height=920, scale=10.0):
        pygame.init()

        self.width = width
        self.height = height
        self.scale = scale
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Bidirectional platoon control simulation")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("arial", 20)
        self.small_font = pygame.font.SysFont("arial", 15)
        self.camera_x = 0.0
        self.camera_smoothness = 0.08

        # layout
        self.sidebar_w = 460
        self.world_x = self.sidebar_w
        self.world_w = self.width - self.sidebar_w
        self.left_clip_margin = 65
        self.left_clip_x = self.world_x + self.left_clip_margin
        self.leader_screen_x = self.world_x + int(self.world_w * 0.85)
        self.road_y = int(self.height * 0.58)
        self.road_height = 120
        self.sky_color = (135, 190, 235)
        self.ground_color = (120, 155, 95)
        self.road_color = (48, 48, 54)
        self.road_edge_color = (210, 210, 210)
        self.lane_color = (245, 230, 120)
        self.pole_color = (80, 80, 80)
        self.panel_bg = (238, 238, 238)
        self.plot_bg = (255, 255, 255)
        self.grid_color = (222, 222, 222)
        self.axis_color = (130, 130, 130)
        self.text_color = (20, 20, 20)
        self.label_bg = (250, 250, 250)
        self.label_border = (150, 150, 150)
        self.sidebar_border = (200, 200, 200)
        self.vehicle_colors = [
            (220, 70, 70),
            (70, 140, 255),
            (90, 200, 160),
            (240, 170, 60),
            (180, 110, 240),
            (255, 120, 170),
        ]

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
        return True

    def tick(self, fps=60):
        self.clock.tick(fps)

    def close(self):
        pygame.quit()

    def draw_scene(self, leader, vehicles, sim_time, leader_cmd, histories, title="Interactive platoon"):
        self._update_camera(leader.p)
        self.screen.fill((230, 230, 230))

        self._draw_sidebar(histories)
        self._draw_world(leader, vehicles, sim_time, leader_cmd, title)

        pygame.display.flip()

    def _update_camera(self, target_x):
        self.camera_x += self.camera_smoothness * (target_x - self.camera_x)

    def world_to_screen_x(self, x_world):
        return int((x_world - self.camera_x) * self.scale + self.leader_screen_x)

    def _visible_world_bounds(self, margin=100.0):
        left_visible_m = (self.leader_screen_x - self.world_x) / self.scale
        right_visible_m = (self.world_x + self.world_w - self.leader_screen_x) / self.scale
        world_start = self.camera_x - left_visible_m - margin
        world_end = self.camera_x + right_visible_m + margin
        return world_start, world_end

    def _draw_sidebar(self, histories):
        sidebar_rect = pygame.Rect(0, 0, self.sidebar_w, self.height)
        pygame.draw.rect(self.screen, self.panel_bg, sidebar_rect)
        pygame.draw.line(self.screen, self.sidebar_border, (self.sidebar_w, 0), (self.sidebar_w, self.height), 2)

        plot_titles = [
            ("Velocity", histories.get("t", []), histories.get("v", []), None),
            ("Acceleration", histories.get("t", []), histories.get("a", []), None),
            ("Command", histories.get("t", []), histories.get("u", []), None),
            ("Spacing", histories.get("t", []), histories.get("d", []), None),
            ("Error", histories.get("t", []), histories.get("e", []), None),
        ]

        margin_x = 14
        margin_top = 14
        margin_bottom = 14
        gap = 10

        usable_h = self.height - margin_top - margin_bottom - gap * (len(plot_titles) - 1)
        plot_h = usable_h // len(plot_titles)
        plot_w = self.sidebar_w - 2 * margin_x

        for i, (title, t_hist, series_hist, y_lim) in enumerate(plot_titles):
            y = margin_top + i * (plot_h + gap)
            rect = pygame.Rect(margin_x, y, plot_w, plot_h)
            self._draw_plot(rect, title, t_hist, series_hist, y_lim)

    def _draw_world(self, leader, vehicles, sim_time, leader_cmd, title):
        world_rect = pygame.Rect(self.world_x, 0, self.world_w, self.height)
        pygame.draw.rect(self.screen, self.sky_color, world_rect)

        pygame.draw.rect(
            self.screen,
            self.ground_color,
            (self.world_x, self.road_y - 130, self.world_w, self.height - (self.road_y - 130)),
        )

        self._draw_world_background()
        self._draw_road()
        self._draw_vehicles(leader, vehicles)
        self._draw_hud(leader, vehicles, sim_time, leader_cmd, title)

    def _draw_world_background(self):
        world_start, world_end = self._visible_world_bounds(margin=100.0)

        pole_spacing = 25.0
        k0 = int(world_start // pole_spacing) - 1
        k1 = int(world_end // pole_spacing) + 1

        horizon_y = self.road_y - self.road_height // 2 - 30

        for k in range(k0, k1 + 1):
            x_world = k * pole_spacing
            x_screen = self.world_to_screen_x(x_world)

            if x_screen < self.left_clip_x - 40 or x_screen > self.world_x + self.world_w + 40:
                continue

            pygame.draw.line(
                self.screen,
                self.pole_color,
                (x_screen, horizon_y - 45),
                (x_screen, horizon_y + 10),
                3,
            )
            pygame.draw.circle(self.screen, (70, 120, 70), (x_screen, horizon_y - 55), 14)

    def _draw_road(self):
        road_top = self.road_y - self.road_height // 2
        road_bottom = self.road_y + self.road_height // 2

        pygame.draw.rect(
            self.screen,
            self.road_color,
            (self.world_x, road_top, self.world_w, self.road_height),
        )
        pygame.draw.line(self.screen, self.road_edge_color, (self.world_x, road_top), (self.world_x + self.world_w, road_top), 3)
        pygame.draw.line(self.screen, self.road_edge_color, (self.world_x, road_bottom), (self.world_x + self.world_w, road_bottom), 3)

        dash_spacing = 12.0
        dash_length = 6.0

        world_start, world_end = self._visible_world_bounds(margin=100.0)
        k0 = int(world_start // dash_spacing) - 2
        k1 = int(world_end // dash_spacing) + 2

        for k in range(k0, k1 + 1):
            x1_world = k * dash_spacing
            x2_world = x1_world + dash_length

            x1 = self.world_to_screen_x(x1_world)
            x2 = self.world_to_screen_x(x2_world)

            if x2 < self.left_clip_x or x1 > self.world_x + self.world_w:
                continue

            pygame.draw.line(self.screen, self.lane_color, (x1, self.road_y), (x2, self.road_y), 5)

    def _draw_vehicles(self, leader, vehicles):
        all_vehicles = [leader] + vehicles
        labels = ["Leader"] + [f"Car {v.id}" for v in vehicles]
        color_cycle = cycle(self.vehicle_colors)
        for vehicle, label, color in zip(all_vehicles, labels, color_cycle):
            self._draw_vehicle(vehicle, color, label)

    def _draw_vehicle(self, vehicle, color, label):
        x = self.world_to_screen_x(vehicle.p)

        body_w = max(40, int(vehicle.length * self.scale))
        body_h = 26
        right_clip_x = self.world_x + self.world_w

        if x + body_w // 2 < self.left_clip_x or x - body_w // 2 > right_clip_x:
            return

        body_rect = pygame.Rect(x - body_w // 2, self.road_y - body_h // 2 - 16, body_w, body_h)
        cabin_rect = pygame.Rect(x - int(body_w * 0.15), body_rect.y - 14, int(body_w * 0.35), 14)

        pygame.draw.rect(self.screen, color, body_rect, border_radius=8)
        pygame.draw.rect(self.screen, color, cabin_rect, border_radius=6)

        wheel_y = body_rect.bottom + 2
        for offset in (-body_w * 0.28, body_w * 0.28):
            pygame.draw.circle(self.screen, (25, 25, 25), (int(x + offset), wheel_y), 6)

        label_text = f"{label} | {vehicle.v:.1f} m/s"
        label_surface = self.small_font.render(label_text, True, (20, 20, 20))
        label_rect = label_surface.get_rect(midbottom=(x, body_rect.y - 8))
        bg_rect = pygame.Rect(label_rect.x - 6, label_rect.y - 3, label_rect.width + 12, label_rect.height + 6)

        pygame.draw.rect(self.screen, self.label_bg, bg_rect, border_radius=6)
        pygame.draw.rect(self.screen, self.label_border, bg_rect, width=1, border_radius=6)
        self.screen.blit(label_surface, label_rect)

    def _draw_hud(self, leader, vehicles, sim_time, leader_cmd, title):
        panel = pygame.Rect(self.world_x + 16, 16, 360, 220)
        pygame.draw.rect(self.screen, self.label_bg, panel, border_radius=10)
        pygame.draw.rect(self.screen, (180, 180, 180), panel, width=2, border_radius=10)

        lines = [
            f"{title}",
            f"time: {sim_time:.2f} s",
            f"leader pos: {leader.p:.2f} m",
            f"leader vel: {leader.v:.2f} m/s",
            f"leader acc: {leader.a:.2f} m/s²",
            f"leader cmd: {leader_cmd:.2f}",
            f"followers: {len(vehicles)}",
            "controls: W accelerate | S brake",
        ]

        for i, txt in enumerate(lines):
            surf = self.font.render(txt, True, self.text_color)
            self.screen.blit(surf, (panel.x + 14, panel.y + 10 + i * surf.get_height()))

    def _draw_plot(self, rect, title, t_hist, series_hist, y_lim=None):
        pygame.draw.rect(self.screen, self.plot_bg, rect, border_radius=8)
        pygame.draw.rect(self.screen, (200, 200, 200), rect, width=1, border_radius=8)

        left_pad = 54
        right_pad = 12
        top_pad = 34
        bottom_pad = 22

        x0 = rect.x + left_pad
        y0 = rect.y + top_pad
        w = rect.width - left_pad - right_pad
        h = rect.height - top_pad - bottom_pad

        pygame.draw.rect(self.screen, (252, 252, 252), (x0, y0, w, h))

        for k in range(5):
            yy = y0 + int(h * k / 4)
            pygame.draw.line(self.screen, self.grid_color, (x0, yy), (x0 + w, yy), 1)

        pygame.draw.line(self.screen, self.axis_color, (x0, y0), (x0, y0 + h), 1)
        pygame.draw.line(self.screen, self.axis_color, (x0, y0 + h), (x0 + w, y0 + h), 1)

        title_surf = self.small_font.render(title, True, self.text_color)
        self.screen.blit(title_surf, (rect.x + 10, rect.y + 4))

        if len(t_hist) < 2:
            empty = self.small_font.render("waiting for data...", True, (140, 140, 140))
            self.screen.blit(empty, (x0 + 12, y0 + 10))
            return

        stride = max(1, len(t_hist) // 400)
        t_sub = t_hist[::stride]
        valid_series = [np.asarray(s[::stride]) for s in series_hist if len(s) > 0]

        if not valid_series:
            empty = self.small_font.render("no series yet", True, (140, 140, 140))
            self.screen.blit(empty, (x0 + 12, y0 + 10))
            return

        t_min = t_sub[0]
        t_max = t_sub[-1]
        if abs(t_max - t_min) < 1e-12:
            return

        if y_lim is None:
            y_min = min(np.min(s) for s in valid_series)
            y_max = max(np.max(s) for s in valid_series)
        else:
            y_min, y_max = y_lim

        if abs(y_max - y_min) < 1e-9:
            y_min -= 1.0
            y_max += 1.0

        def sx(tv):
            return x0 + int((tv - t_min) / (t_max - t_min) * w)

        def sy(val):
            return y0 + h - int((val - y_min) / (y_max - y_min) * h)

        y_max_label = self.small_font.render(f"{y_max:.1f}", True, (90, 90, 90))
        y_min_label = self.small_font.render(f"{y_min:.1f}", True, (90, 90, 90))
        t_min_label = self.small_font.render(f"{t_min:.1f}", True, (90, 90, 90))
        t_max_label = self.small_font.render(f"{t_max:.1f}", True, (90, 90, 90))

        self.screen.blit(y_max_label, (rect.x + 6, y0 - 8))
        self.screen.blit(y_min_label, (rect.x + 6, y0 + h - 8))
        self.screen.blit(t_min_label, (x0, y0 + h + 2))
        self.screen.blit(t_max_label, (x0 + w - t_max_label.get_width(), y0 + h + 2))

        for idx, s in enumerate(valid_series):
            if len(s) < 2:
                continue

            color = self.vehicle_colors[idx % len(self.vehicle_colors)]
            n = min(len(t_sub), len(s))
            pts = [(sx(t_sub[j]), sy(s[j])) for j in range(n)]
            if len(pts) >= 2:
                pygame.draw.lines(self.screen, color, False, pts, 2)