# Chạy lệnh: pip install -r requirements.txt để cài đặt các thư viện trước khi chạy file
# Chạy lệnh: python app.py để khởi động Webdemo
# Truy cập: http://127.0.0.1:5000/ để vào Web

from flask import Flask, render_template, jsonify, request
import osmnx as ox
import networkx as nx
import folium
from math import radians, sin, cos, sqrt, atan2
import heapq
import time
import logging

app = Flask(__name__)
logging.basicConfig(level=logging.DEBUG)

# Đọc và lưu đồ thị để tái sử dụng
try:
    G = ox.graph_from_xml('map.osm')
    app.logger.info("Đã đọc thành công file map.osm")
except Exception as e:
    app.logger.error(f"Lỗi khi đọc file map.osm: {str(e)}")
    G = None

def haversine_distance(lat1, lon1, lat2, lon2):
    """Tính khoảng cách giữa 2 điểm dựa trên tọa độ địa lý"""
    R = 6371
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    return R * c

def a_star_step(G, start_node, end_node):
    """Generator cho từng bước của thuật toán A*"""
    try:
        frontier = []
        heapq.heappush(frontier, (0, start_node))
        came_from = {start_node: None}
        cost_so_far = {start_node: 0}
        current_path = []
        
        while frontier:
            current = heapq.heappop(frontier)[1]
            current_path.append(current)
            
            if current == end_node:
                path = []
                current = end_node
                while current is not None:
                    path.append(current)
                    current = came_from.get(current)
                path.reverse()
                yield {'status': 'done', 'path': path, 'explored': current_path}
                break
                
            for next_node in G.neighbors(current):
                edge_data = G.get_edge_data(current, next_node)
                if len(edge_data) > 0:
                    edge = edge_data[0]
                    new_cost = cost_so_far[current] + edge.get('length', 0)
                    
                    if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                        cost_so_far[next_node] = new_cost
                        priority = new_cost + haversine_distance(
                            G.nodes[next_node]['y'], G.nodes[next_node]['x'],
                            G.nodes[end_node]['y'], G.nodes[end_node]['x']
                        )
                        heapq.heappush(frontier, (priority, next_node))
                        came_from[next_node] = current
                        yield {
                            'status': 'exploring',
                            'current': current,
                            'next': next_node,
                            'explored': current_path
                        }
    except Exception as e:
        app.logger.error(f"Lỗi trong a_star_step: {str(e)}")
        yield {'status': 'error', 'message': str(e)}

@app.route('/')
def index():
    try:
        # Tạo bản đồ ban đầu
        center_lat = 21.0285  # Tọa độ trung tâm Hà Nội
        center_lon = 105.8542
        m = folium.Map(location=[center_lat, center_lon], zoom_start=14)
        return render_template('index.html', map=m._repr_html_())
    except Exception as e:
        app.logger.error(f"Lỗi trong route /: {str(e)}")
        return jsonify({'error': str(e)}), 500

@app.route('/find_path', methods=['POST'])
def find_path():
    try:
        if not G:
            return jsonify({'error': 'Đồ thị chưa được khởi tạo'}), 500

        data = request.json
        app.logger.info(f"Nhận request tìm đường với data: {data}")
        
        start_coords = data['start']
        end_coords = data['end']
        
        # Tìm node gần nhất
        start_node = ox.nearest_nodes(G, start_coords[1], start_coords[0])
        end_node = ox.nearest_nodes(G, end_coords[1], end_coords[0])
        
        app.logger.info(f"Đã tìm thấy nodes: start={start_node}, end={end_node}")
        
        # Tạo generator cho thuật toán A*
        path_generator = a_star_step(G, start_node, end_node)
        
        # Lưu trạng thái để stream về client
        app.path_generator = path_generator
        return jsonify({'status': 'started'})
    except Exception as e:
        app.logger.error(f"Lỗi trong route /find_path: {str(e)}")
        return jsonify({'error': str(e)}), 500

@app.route('/get_next_step')
def get_next_step():
    try:
        if not hasattr(app, 'path_generator'):
            return jsonify({'error': 'Chưa bắt đầu tìm đường'}), 400
            
        step_data = next(app.path_generator)
        app.logger.debug(f"Bước tiếp theo: {step_data['status']}")
        
        result = {
            'status': step_data['status'],
            'points': []
        }
        
        if step_data['status'] == 'exploring':
            current = step_data['current']
            next_node = step_data['next']
            result['points'] = [
                [G.nodes[current]['y'], G.nodes[current]['x']],
                [G.nodes[next_node]['y'], G.nodes[next_node]['x']]
            ]
        elif step_data['status'] == 'done':
            result['points'] = [[G.nodes[n]['y'], G.nodes[n]['x']] 
                              for n in step_data['path']]
        elif step_data['status'] == 'error':
            return jsonify({'error': step_data['message']}), 500
            
        return jsonify(result)
    except StopIteration:
        return jsonify({'status': 'finished'})
    except Exception as e:
        app.logger.error(f"Lỗi trong route /get_next_step: {str(e)}")
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    app.run(debug=True) 
    