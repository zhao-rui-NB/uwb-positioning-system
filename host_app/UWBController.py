from dataclasses import dataclass
from typing import Literal, Optional
from SerialWorker import SerialWorker
import json
import time


# === Dataclasses ===
@dataclass
class RangeResponse:
    node_a_id: int
    node_b_id: int
    rssi_dbm: float
    distance_m: float

    @classmethod
    def from_json(cls, data: dict) -> Optional["RangeResponse"]:
        event = data.get("event", "")
        if event not in ("range_report", "range_final"):
            return None
        try:
            return cls(
                node_a_id=data["node_a_id"],
                node_b_id=data["node_b_id"],
                rssi_dbm=data["rssi_dbm"],
                distance_m=data["distance_m"],
            )
        except KeyError:
            return None


@dataclass
class PingResponse:
    node_id: int
    system_state: int
    voltage_mv: int

    @classmethod
    def from_json(cls, data: dict) -> Optional["PingResponse"]:
        if data.get("event") != "ping_resp":
            return None
        try:
            return cls(
                node_id=data["node_id"],
                system_state=data["system_state"],
                voltage_mv=data["voltage_mv"],
            )
        except KeyError:
            return None


# === Controller ===
class UWBController:
    def __init__(self, serial_worker: SerialWorker, debug_print=True):
        self.serial_worker = serial_worker
        self.debug_print = debug_print

    def _debug(self, msg: str):
        if self.debug_print:
            print(f"[UWBController] {msg}")

    def calc_node_id(self, role: Literal["anchor", "tag"], node_id: int) -> int:
        base = 0xFF00 if role == "anchor" else 0x0000
        return base + node_id

    def ping(self, node_id: int, timeout=0.1) -> Optional[PingResponse]:
        cmd = f"ping {node_id}"
        self.serial_worker.send_command(cmd)
        self._debug(f"Sent command: {cmd}")

        data = self.serial_worker.read_response(timeout=timeout)
        if not data:
            self._debug("⚠️ ping timeout")
            return None

        ping = PingResponse.from_json(data)
        if ping:
            self._debug(f"✅ ping_resp: {ping}")
        return ping
    
    def ping_multiple(self, node_ids: list[int], timeout=0.1) -> list[Optional[PingResponse]]:
        responses = []
        for node_id in node_ids:
            response = self.ping(node_id, timeout=timeout)
            responses.append(response)
        return responses

    def trigger(self, initiator_id: int, responder_id: int, timeout=0.1) -> Optional[RangeResponse]:
        cmd = f"trigger {initiator_id} {responder_id}"
        self.serial_worker.send_command(cmd)
        self._debug(f"Sent command: {cmd}")

        data = self.serial_worker.read_response(timeout=timeout)
        if not data:
            self._debug("⚠️ range timeout")
            return None

        report = RangeResponse.from_json(data)
        if report:
            self._debug(
                f"📡 range: {report.node_a_id}↔{report.node_b_id} "
                f"{report.distance_m:.2f}m RSSI={report.rssi_dbm:.1f}dBm"
            )
        return report
    
    def trigger_multiple(self, initiator_id: int, responder_ids: list[int], timeout=0.1) -> list[Optional[RangeResponse]]:
        reports = []
        for responder_id in responder_ids:
            report = self.trigger(initiator_id, responder_id, timeout=timeout)
            reports.append(report)
        return reports
        
    
    


# === Example main ===
if __name__ == "__main__":
    COM = "COM18"
    worker = SerialWorker(port=COM, baudrate=115200, debug_print=True)
    worker.open()

    controller = UWBController(serial_worker=worker, debug_print=False)

    anchor_ids = [0xFF00, 0xFF01, 0xFF02]
    tag_ids = [0x0000]

    last_fps_time = time.time()
    frame_count = 0

    try:
        while True:
            for tag_id in tag_ids:
                rssi_list, dist_list = [], []
                for anchor_id in anchor_ids:
                    report = controller.trigger(tag_id, anchor_id)
                    rssi_list.append(report.rssi_dbm if report else -1)
                    dist_list.append(report.distance_m if report else -1)
                    
                    # report2 = controller.ping(anchor_id)
                    # if report2:
                    #     print(f"   🔋 Anchor {anchor_id}: Voltage={report2.voltage_mv}mV State={report2.system_state}")

                print(f"📡 Tag {tag_id}: RSSI={rssi_list}, Dist={dist_list}")

            # FPS monitor
            frame_count += 1
            if time.time() - last_fps_time >= 1.0:
                fps = frame_count / (time.time() - last_fps_time)
                print(f"🔄 FPS: {fps:.2f}")
                frame_count = 0
                last_fps_time = time.time()

    except KeyboardInterrupt:
        print("Exiting...")

    worker.close()
