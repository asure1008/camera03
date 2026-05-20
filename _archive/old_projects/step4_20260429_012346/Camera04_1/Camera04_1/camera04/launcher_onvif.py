from __future__ import annotations

import datetime
import json
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from dataclasses import dataclass


SOAP12_ENV = "http://www.w3.org/2003/05/soap-envelope"
SOAP11_ENV = "http://schemas.xmlsoap.org/soap/envelope/"
TT_NS = "http://www.onvif.org/ver10/schema"
TDS_NS = "http://www.onvif.org/ver10/device/wsdl"
TRT_NS = "http://www.onvif.org/ver10/media/wsdl"
TPZ_NS = "http://www.onvif.org/ver10/ptz/wsdl"


@dataclass
class OnvifHttpResponse:
    body: bytes
    status_code: int = 200


def soap_wrap(body_xml: str) -> bytes:
    return (
        '<?xml version="1.0" encoding="utf-8"?>'
        f'<s:Envelope xmlns:s="{SOAP12_ENV}">'
        f"<s:Body>{body_xml}</s:Body>"
        "</s:Envelope>"
    ).encode("utf-8")


def soap_fault(reason: str, status_code: int = 503) -> OnvifHttpResponse:
    body = (
        f'<s:Fault xmlns:s="{SOAP12_ENV}">'
        "<s:Code><s:Value>s:Receiver</s:Value></s:Code>"
        f"<s:Reason><s:Text xml:lang=\"en\">{reason}</s:Text></s:Reason>"
        "</s:Fault>"
    )
    return OnvifHttpResponse(body=soap_wrap(body), status_code=status_code)


def parse_soap_action(raw: bytes) -> tuple[str | None, ET.Element | None]:
    try:
        root = ET.fromstring(raw)
    except Exception:
        return None, None

    for ns in (SOAP12_ENV, SOAP11_ENV):
        body = root.find(f"{{{ns}}}Body")
        if body is None:
            continue
        for child in body:
            tag = child.tag
            local = tag.split("}")[-1] if "}" in tag else tag
            return local, child
    return None, None


def find_elem(elem: ET.Element | None, local_name: str) -> ET.Element | None:
    if elem is None:
        return None
    for child in elem.iter():
        tag = child.tag.split("}")[-1] if "}" in child.tag else child.tag
        if tag == local_name:
            return child
    return None


def find_text(elem: ET.Element | None, local_name: str) -> str:
    found = find_elem(elem, local_name)
    return (found.text or "").strip() if found is not None else ""


class OnvifService:
    def __init__(self, isaac_port: int) -> None:
        self.isaac_port = isaac_port
        self.pan_limit = 90.0
        self.tilt_min = -90.0
        self.tilt_max = 90.0
        self.presets = {
            "1": (0.0, 0.0, 1.0),
            "2": (90.0, 0.0, 1.0),
            "3": (-90.0, 0.0, 1.0),
            "home": (0.0, 0.0, 1.0),
        }

    def _call_control(self, *, pan: float | None = None, tilt: float | None = None, zoom: float | None = None) -> tuple[bool, str | None]:
        payload: dict[str, float] = {}
        if pan is not None:
            payload["pan"] = round(float(pan), 4)
        if tilt is not None:
            payload["tilt"] = round(float(tilt), 4)
        if zoom is not None:
            payload["zoom"] = round(float(zoom), 4)
        if not payload:
            return True, None

        request = urllib.request.Request(
            f"http://127.0.0.1:{self.isaac_port}/control",
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=3) as response:
                body = json.loads(response.read() or b"{}")
        except urllib.error.HTTPError as exc:
            details = exc.read().decode("utf-8", errors="replace")
            return False, details or f"HTTP {exc.code}"
        except Exception as exc:
            return False, str(exc)

        if body.get("ok", True):
            return True, None
        return False, body.get("error", "PTZ control rejected")

    def _get_current_ptz(self) -> tuple[tuple[float, float, float] | None, str | None]:
        try:
            with urllib.request.urlopen(
                f"http://127.0.0.1:{self.isaac_port}/status",
                timeout=1,
            ) as response:
                data = json.loads(response.read() or b"{}")
        except urllib.error.HTTPError as exc:
            details = exc.read().decode("utf-8", errors="replace")
            return None, details or f"HTTP {exc.code}"
        except Exception as exc:
            return None, str(exc)

        return (
            (
                float(data.get("pan", 0.0)),
                float(data.get("tilt", 0.0)),
                float(data.get("zoom", 1.0)),
            ),
            None,
        )

    def handle_device_service(self, xml_body: bytes, host_port: str) -> OnvifHttpResponse:
        action, _ = parse_soap_action(xml_body)

        if action == "GetCapabilities":
            body = f"""
<tds:GetCapabilitiesResponse xmlns:tds="{TDS_NS}">
  <tds:Capabilities>
    <tt:Device xmlns:tt="{TT_NS}">
      <tt:XAddr>http://{host_port}/onvif/device_service</tt:XAddr>
    </tt:Device>
    <tt:Media xmlns:tt="{TT_NS}">
      <tt:XAddr>http://{host_port}/onvif/media_service</tt:XAddr>
      <tt:StreamingCapabilities>
        <tt:RTPMulticast>false</tt:RTPMulticast>
        <tt:RTP_TCP>false</tt:RTP_TCP>
        <tt:RTP_RTSP_TCP>false</tt:RTP_RTSP_TCP>
      </tt:StreamingCapabilities>
    </tt:Media>
    <tt:PTZ xmlns:tt="{TT_NS}">
      <tt:XAddr>http://{host_port}/onvif/ptz_service</tt:XAddr>
    </tt:PTZ>
  </tds:Capabilities>
</tds:GetCapabilitiesResponse>"""
            return OnvifHttpResponse(body=soap_wrap(body))

        if action == "GetSystemDateAndTime":
            now = datetime.datetime.utcnow()
            body = f"""
<tds:GetSystemDateAndTimeResponse xmlns:tds="{TDS_NS}">
  <tds:SystemDateAndTime>
    <tt:DateTimeType xmlns:tt="{TT_NS}">NTP</tt:DateTimeType>
    <tt:DaylightSavings xmlns:tt="{TT_NS}">false</tt:DaylightSavings>
    <tt:UTCDateTime xmlns:tt="{TT_NS}">
      <tt:Time>
        <tt:Hour>{now.hour}</tt:Hour>
        <tt:Minute>{now.minute}</tt:Minute>
        <tt:Second>{now.second}</tt:Second>
      </tt:Time>
      <tt:Date>
        <tt:Year>{now.year}</tt:Year>
        <tt:Month>{now.month}</tt:Month>
        <tt:Day>{now.day}</tt:Day>
      </tt:Date>
    </tt:UTCDateTime>
  </tds:SystemDateAndTime>
</tds:GetSystemDateAndTimeResponse>"""
            return OnvifHttpResponse(body=soap_wrap(body))

        if action == "GetServices":
            body = f"""
<tds:GetServicesResponse xmlns:tds="{TDS_NS}">
  <tds:Service>
    <tds:Namespace>{TRT_NS}</tds:Namespace>
    <tds:XAddr>http://{host_port}/onvif/media_service</tds:XAddr>
    <tds:Version><tt:Major xmlns:tt="{TT_NS}">2</tt:Major><tt:Minor xmlns:tt="{TT_NS}">0</tt:Minor></tds:Version>
  </tds:Service>
  <tds:Service>
    <tds:Namespace>{TPZ_NS}</tds:Namespace>
    <tds:XAddr>http://{host_port}/onvif/ptz_service</tds:XAddr>
    <tds:Version><tt:Major xmlns:tt="{TT_NS}">2</tt:Major><tt:Minor xmlns:tt="{TT_NS}">0</tt:Minor></tds:Version>
  </tds:Service>
</tds:GetServicesResponse>"""
            return OnvifHttpResponse(body=soap_wrap(body))

        return OnvifHttpResponse(body=soap_wrap(f'<tds:UnknownResponse xmlns:tds="{TDS_NS}"/>'))

    def handle_media_service(self, xml_body: bytes, host_port: str) -> OnvifHttpResponse:
        action, _ = parse_soap_action(xml_body)

        if action == "GetProfiles":
            body = f"""
<trt:GetProfilesResponse xmlns:trt="{TRT_NS}">
  <trt:Profiles token="MainProfileToken" fixed="true" xmlns:tt="{TT_NS}">
    <tt:Name>MainStream</tt:Name>
    <tt:PTZConfiguration token="PTZConfigToken">
      <tt:Name>PTZConfig</tt:Name>
      <tt:UseCount>1</tt:UseCount>
      <tt:NodeToken>PTZNodeToken</tt:NodeToken>
      <tt:PanTiltLimits>
        <tt:Range>
          <tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace</tt:URI>
          <tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>
          <tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>
        </tt:Range>
      </tt:PanTiltLimits>
      <tt:ZoomLimits>
        <tt:Range>
          <tt:URI>http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace</tt:URI>
          <tt:XRange><tt:Min>0</tt:Min><tt:Max>1</tt:Max></tt:XRange>
        </tt:Range>
      </tt:ZoomLimits>
    </tt:PTZConfiguration>
  </trt:Profiles>
</trt:GetProfilesResponse>"""
            return OnvifHttpResponse(body=soap_wrap(body))

        if action == "GetSnapshotUri":
            body = f"""
<trt:GetSnapshotUriResponse xmlns:trt="{TRT_NS}">
  <trt:MediaUri>
    <tt:Uri xmlns:tt="{TT_NS}">http://{host_port}/onvif-snap.jpg</tt:Uri>
    <tt:InvalidAfterConnect xmlns:tt="{TT_NS}">false</tt:InvalidAfterConnect>
    <tt:InvalidAfterReboot xmlns:tt="{TT_NS}">false</tt:InvalidAfterReboot>
    <tt:Timeout xmlns:tt="{TT_NS}">PT30S</tt:Timeout>
  </trt:MediaUri>
</trt:GetSnapshotUriResponse>"""
            return OnvifHttpResponse(body=soap_wrap(body))

        if action == "GetServiceCapabilities":
            body = f"""
<trt:GetServiceCapabilitiesResponse xmlns:trt="{TRT_NS}">
  <trt:Capabilities SnapshotUri="true" Rotation="false" VideoSourceMode="false"
                    OSD="false" TemporaryOSDText="false" EXICompression="false"/>
</trt:GetServiceCapabilitiesResponse>"""
            return OnvifHttpResponse(body=soap_wrap(body))

        return OnvifHttpResponse(body=soap_wrap(f'<trt:UnknownResponse xmlns:trt="{TRT_NS}"/>'))

    def handle_ptz_service(self, xml_body: bytes) -> OnvifHttpResponse:
        action, elem = parse_soap_action(xml_body)

        if action == "GetNodes":
            body = f"""
<tptz:GetNodesResponse xmlns:tptz="{TPZ_NS}">
  <tptz:PTZNode token="PTZNodeToken" FixedHomePosition="false" xmlns:tt="{TT_NS}">
    <tt:Name>PTZNode</tt:Name>
    <tt:SupportedPTZSpaces>
      <tt:AbsolutePanTiltPositionSpace>
        <tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace</tt:URI>
        <tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>
        <tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>
      </tt:AbsolutePanTiltPositionSpace>
      <tt:AbsoluteZoomPositionSpace>
        <tt:URI>http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace</tt:URI>
        <tt:XRange><tt:Min>0</tt:Min><tt:Max>1</tt:Max></tt:XRange>
      </tt:AbsoluteZoomPositionSpace>
      <tt:RelativePanTiltTranslationSpace>
        <tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/TranslationGenericSpace</tt:URI>
        <tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>
        <tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>
      </tt:RelativePanTiltTranslationSpace>
    </tt:SupportedPTZSpaces>
    <tt:MaximumNumberOfPresets>10</tt:MaximumNumberOfPresets>
    <tt:HomeSupported>false</tt:HomeSupported>
  </tptz:PTZNode>
</tptz:GetNodesResponse>"""
            return OnvifHttpResponse(body=soap_wrap(body))

        if action == "GetConfigurations":
            body = f"""
<tptz:GetConfigurationsResponse xmlns:tptz="{TPZ_NS}">
  <tptz:PTZConfiguration token="PTZConfigToken" xmlns:tt="{TT_NS}">
    <tt:Name>PTZConfig</tt:Name>
    <tt:UseCount>1</tt:UseCount>
    <tt:NodeToken>PTZNodeToken</tt:NodeToken>
    <tt:DefaultAbsolutePanTiltPositionSpace>http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace</tt:DefaultAbsolutePanTiltPositionSpace>
    <tt:DefaultAbsoluteZoomPositionSpace>http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace</tt:DefaultAbsoluteZoomPositionSpace>
    <tt:PanTiltLimits>
      <tt:Range>
        <tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace</tt:URI>
        <tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>
        <tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>
      </tt:Range>
    </tt:PanTiltLimits>
    <tt:ZoomLimits>
      <tt:Range>
        <tt:URI>http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace</tt:URI>
        <tt:XRange><tt:Min>0</tt:Min><tt:Max>1</tt:Max></tt:XRange>
      </tt:Range>
    </tt:ZoomLimits>
  </tptz:PTZConfiguration>
</tptz:GetConfigurationsResponse>"""
            return OnvifHttpResponse(body=soap_wrap(body))

        if action == "GetServiceCapabilities":
            body = f"""
<tptz:GetServiceCapabilitiesResponse xmlns:tptz="{TPZ_NS}">
  <tptz:Capabilities EFlip="false" Reverse="false" GetCompatibleConfigurations="false"
                     MoveStatus="false" StatusPosition="false"/>
</tptz:GetServiceCapabilitiesResponse>"""
            return OnvifHttpResponse(body=soap_wrap(body))

        if action == "GetPresets":
            presets_xml = []
            for token, (pan_deg, tilt_deg, zoom_x) in self.presets.items():
                presets_xml.append(
                    f"""
  <tptz:Preset token="{token}" xmlns:tt="{TT_NS}">
    <tt:Name>Preset{token}</tt:Name>
      <tt:PTZPosition>
      <tt:PanTilt x="{round(pan_deg / self.pan_limit, 4)}" y="{round(tilt_deg / 90.0, 4)}" space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace"/>
      <tt:Zoom x="{round((zoom_x - 1.0) / 31.0, 4)}" space="http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace"/>
    </tt:PTZPosition>
  </tptz:Preset>"""
                )
            body = f'<tptz:GetPresetsResponse xmlns:tptz="{TPZ_NS}">{"".join(presets_xml)}</tptz:GetPresetsResponse>'
            return OnvifHttpResponse(body=soap_wrap(body))

        if action == "AbsoluteMove":
            try:
                pt_elem = find_elem(elem, "PanTilt")
                zoom_elem = find_elem(elem, "Zoom")
                pan_onvif = float(pt_elem.get("x", 0.0)) if pt_elem is not None else 0.0
                tilt_onvif = float(pt_elem.get("y", 0.0)) if pt_elem is not None else 0.0
                zoom_onvif = float(zoom_elem.get("x", 0.0)) if zoom_elem is not None else 0.0
            except Exception as exc:
                return soap_fault(f"Invalid AbsoluteMove payload: {exc}", status_code=400)

            ok, error = self._call_control(
                pan=pan_onvif * self.pan_limit,
                tilt=max(self.tilt_min, min(self.tilt_max, tilt_onvif * 90.0)),
                zoom=1.0 + zoom_onvif * 31.0,
            )
            if not ok:
                return soap_fault(f"AbsoluteMove failed: {error}")
            return OnvifHttpResponse(body=soap_wrap(f'<tptz:AbsoluteMoveResponse xmlns:tptz="{TPZ_NS}"/>'))

        if action == "RelativeMove":
            try:
                pt_elem = find_elem(elem, "PanTilt")
                pan_onvif = float(pt_elem.get("x", 0.0)) if pt_elem is not None else 0.0
                tilt_onvif = float(pt_elem.get("y", 0.0)) if pt_elem is not None else 0.0
            except Exception as exc:
                return soap_fault(f"Invalid RelativeMove payload: {exc}", status_code=400)

            current, error = self._get_current_ptz()
            if current is None:
                return soap_fault(f"RelativeMove failed: {error}")
            current_pan, current_tilt, _ = current

            ok, error = self._call_control(
                pan=max(-self.pan_limit, min(self.pan_limit, current_pan + pan_onvif * self.pan_limit)),
                tilt=max(self.tilt_min, min(self.tilt_max, current_tilt + tilt_onvif * 90.0)),
            )
            if not ok:
                return soap_fault(f"RelativeMove failed: {error}")
            return OnvifHttpResponse(body=soap_wrap(f'<tptz:RelativeMoveResponse xmlns:tptz="{TPZ_NS}"/>'))

        if action == "GotoPreset":
            preset_token = find_text(elem, "PresetToken")
            if preset_token not in self.presets:
                return soap_fault(f"Unknown preset token: {preset_token}", status_code=400)
            pan_deg, tilt_deg, zoom_x = self.presets[preset_token]
            ok, error = self._call_control(pan=pan_deg, tilt=tilt_deg, zoom=zoom_x)
            if not ok:
                return soap_fault(f"GotoPreset failed: {error}")
            return OnvifHttpResponse(body=soap_wrap(f'<tptz:GotoPresetResponse xmlns:tptz="{TPZ_NS}"/>'))

        if action == "GetStatus":
            current, error = self._get_current_ptz()
            if current is None:
                return soap_fault(f"GetStatus failed: {error}")
            cur_pan, cur_tilt, cur_zoom = current
            body = f"""
<tptz:GetStatusResponse xmlns:tptz="{TPZ_NS}">
  <tptz:PTZStatus xmlns:tt="{TT_NS}">
    <tt:Position>
      <tt:PanTilt x="{round(cur_pan / self.pan_limit, 4)}" y="{round(cur_tilt / 90.0, 4)}" space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace"/>
      <tt:Zoom x="{round((cur_zoom - 1.0) / 31.0, 4)}" space="http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace"/>
    </tt:Position>
    <tt:MoveStatus>
      <tt:PanTilt>IDLE</tt:PanTilt>
      <tt:Zoom>IDLE</tt:Zoom>
    </tt:MoveStatus>
  </tptz:PTZStatus>
</tptz:GetStatusResponse>"""
            return OnvifHttpResponse(body=soap_wrap(body))

        return OnvifHttpResponse(body=soap_wrap(f'<tptz:UnknownResponse xmlns:tptz="{TPZ_NS}"/>'))
