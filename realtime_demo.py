import serial
import time
import ast
import numpy as np
import cv2
import sys
import pickle
from tsmoothie.smoother import KalmanSmoother
# Wir importieren DetectingProcess, da du es unten nutzt
from functions2 import PrePipeline, TrackingDetectingMergeProcess, ROIPooling, SubpageInterpolating, DetectingProcess 

# --- KONFIGURATION ---
SERIAL_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200  # Wir nutzen erst mal 115200 für Stabilität!

def initialize_uart(port=SERIAL_PORT, baud_rate=BAUD_RATE):
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        if not ser.is_open:
            raise RuntimeError(f"Failed to open serial port {port}")
        print(f"✅ Reading data from {port} at {baud_rate} baud")
        time.sleep(0.5)
        ser.reset_input_buffer()
        return ser
    except serial.SerialException as e:
        print(f"❌ ERROR: Could not open serial port {port}")
        print(f"   Details: {e}")
        sys.exit(1)

def preprocess_temperature_data(data_str):
    try:
        # Robustes Parsing: JSON oder AST
        try:
            dict_data = ast.literal_eval(data_str)
        except:
            import json
            dict_data = json.loads(data_str)
            
        temperature = np.array(dict_data["temperature"]).reshape((24, 32))
        ambient_temp = dict_data["at"]
        return temperature, ambient_temp
    except Exception:
        return None, None

def apply_color_map(matrix, expansion_coefficient, upper_bound, resize_dim):
    # Min/Max Normalisierung mit Schutz vor Division durch Null
    t_min = np.min(matrix)
    denom = upper_bound - t_min
    if denom == 0: denom = 1
    
    norm = ((matrix - t_min) / denom) * 255
    expanded = np.repeat(np.repeat(norm, expansion_coefficient, axis=0), expansion_coefficient, axis=1)
    colored = cv2.applyColorMap(expanded.astype(np.uint8), cv2.COLORMAP_JET)
    return cv2.resize(colored, resize_dim)

def smooth_predictions(buffer, smoother, predict, max_len=10):
    if len(buffer) >= max_len:
        buffer.pop(0)
    buffer.append(predict)  
    smoother.smooth(buffer)
    smoothed_pred = smoother.smooth_data[0]
    return np.mean(smoothed_pred[-min(max_len, len(smoothed_pred)):])

def main():
    try:
        range_estimator = pickle.load(open('Models/hgbr_range2.sav', 'rb'))
    except FileNotFoundError:
        print("❌ FEHLER: 'Models/hgbr_range2.sav' nicht gefunden!")
        return

    ser = initialize_uart()
    expansion_coefficient = 20
    temperature_upper_bound = 37
    valid_region_area_limit = 10
    data_shape = (24, 32)
    resize_dim = (640, 480)

    prepipeline = PrePipeline(expansion_coefficient, temperature_upper_bound, buffer_size=10, data_shape=data_shape)
    stage1procerss = TrackingDetectingMergeProcess(expansion_coefficient, valid_region_area_limit)
    roipooling = ROIPooling((200, 400), 100, 100)
    
    kalman_smoother = KalmanSmoother(component='level_trend', component_noise={'level': 0.0001, 'trend': 0.01})
    buffer_pred = {}

    print("Start TAdar System!!")

    try:
        while True:
            try:
                data = ser.readline()
                if not data: continue
                msg_str = data.decode('utf-8', errors='ignore').strip()
            except:
                continue
            
            if msg_str:
                sensor_mat, sensor_at = preprocess_temperature_data(msg_str)
                if sensor_mat is None:
                    continue

                ira_img, _, ira_mat = prepipeline.Forward(np.flip(sensor_mat, 0), sensor_at)
                if not isinstance(ira_img, np.ndarray):
                    continue

                # Hier wird 'mask' definiert (1. Rückgabewert)
                mask, _, filtered_mask_colored, _, _, _, valid_BBoxes, valid_timers = stage1procerss.Forward(ira_img)
                
                sub_interp = SubpageInterpolating(np.flip(sensor_mat, 0))
                ira_colored = apply_color_map(sub_interp, expansion_coefficient, temperature_upper_bound, resize_dim)

                depth_map = np.zeros_like(filtered_mask_colored, dtype=float)
                
                for idx, (x, y, w, h) in enumerate(valid_BBoxes):
                    if not (100 < (x + w / 2) < 500):
                        continue

                    roi_t = ira_mat[y:y + h, x:x + w]
                    pooled_roi = roipooling.PoolingNumpy(roi_t)
                    
                    flat_roi = pooled_roi.flatten()
                    if len(flat_roi) < 8: continue
                    
                    input_data = np.concatenate([np.sort(flat_roi)[::-1][:8], [x + w / 2, y + h / 2]])

                    predict_r = range_estimator.predict(input_data.reshape(1, -1))[0]
                    if idx in buffer_pred:
                        predict = smooth_predictions(buffer_pred[idx], kalman_smoother, predict_r)
                    else:
                        buffer_pred[idx] = [predict_r]
                        predict = predict_r

                    cv2.rectangle(ira_colored, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(ira_colored, f"{round(predict, 2)}m", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,  color=(0, 0, 255),  fontScale=1.2, thickness = 3)

                    center_pt = (int(y + h / 2), int(x + w / 2))
                    
                    # --- HIER WAR DER FEHLER ---
                    # Wir übergeben jetzt 'mask' als zweiten Parameter!
                    try:
                        divs = DetectingProcess.RegionDivid(filtered_mask_colored, mask)
                        if divs is not None:
                            for m in divs:
                                if m[center_pt[0], center_pt[1]] > 0.1:
                                    depth_map += m * predict
                                    break
                    except AttributeError:
                        # Fallback falls Import falsch ist
                        pass
                    except TypeError:
                        # Fallback falls Signatur doch anders ist
                        pass

                depth_map = np.where(depth_map < 0.1, 4.5, depth_map)
                depth_colormap = cv2.applyColorMap(((depth_map / 4.5) * 255).astype(np.uint8), cv2.COLORMAP_JET)
                combined_image = np.hstack((ira_colored, depth_colormap))

                cv2.imshow('TAdar Realtime', combined_image)
                if cv2.waitKey(1) & 0xFF in {27, 113}:
                    break

        ser.close()
        cv2.destroyAllWindows()
    except KeyboardInterrupt:
        ser.close()
        cv2.destroyAllWindows()
        print("Process interrupted by user")

if __name__ == "__main__":
    main()