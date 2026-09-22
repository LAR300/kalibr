#!/usr/bin/env python3
"""Converte um bag ROS 2 (MCAP/sqlite3/.zip) em um bag ROS 1 para o Kalibr.

- Mantém apenas os tópicos de imagem estéreo + IMU (exclui tipos custom como dvl_msgs/petro).
- Converte as imagens para mono8 (cinza) -> bag muito menor e sem risco de bgra8 no Kalibr.
- Preserva os timestamps do header.

Roda no HOST (lib `rosbags`, sem ROS instalado). Ver .ai/specs/validacao-calibracao-tanque (D1).

Uso:
  python3 ros2_to_ros1_kalibr.py <bag> -o saida.bag \\
    --image-topics /zed/zed_node/left/color/raw/image /zed/zed_node/right/color/raw/image \\
    --imu-topics /imu/data
  # opcional: --limit N (teste rápido)
"""
from __future__ import print_function
import argparse
import sys
import zipfile
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

TS1 = get_typestore(Stores.ROS1_NOETIC)
Image = TS1.types["sensor_msgs/msg/Image"]
Imu = TS1.types["sensor_msgs/msg/Imu"]
Header = TS1.types["std_msgs/msg/Header"]
Time = TS1.types["builtin_interfaces/msg/Time"]
Quaternion = TS1.types["geometry_msgs/msg/Quaternion"]
Vector3 = TS1.types["geometry_msgs/msg/Vector3"]


def resolve_bag(path):
    p = Path(path)
    if p.is_dir():
        return p
    if p.suffix == ".zip":
        outdir = p.with_suffix("")
        if not (outdir / "metadata.yaml").exists():
            outdir.mkdir(parents=True, exist_ok=True)
            print("Descompactando {0}...".format(p))
            with zipfile.ZipFile(p) as zf:
                zf.extractall(outdir)
        if (outdir / "metadata.yaml").exists():
            return outdir
        for md in outdir.rglob("metadata.yaml"):
            return md.parent
        return outdir
    if p.suffix == ".mcap":
        return p.parent
    raise SystemExit("bag não reconhecido: {0}".format(path))


def to_mono8(msg):
    """Converte a imagem para mono8 (uint8, 1 canal)."""
    h, w, enc = msg.height, msg.width, msg.encoding
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ("mono8",):
        return buf.reshape(h, w)
    if enc in ("bgra8", "rgba8", "bgr8", "rgb8"):
        ch = 4 if enc.endswith("a8") else 3
        arr = buf.reshape(h, w, ch).astype(np.float32)
        if enc.startswith("bgr"):
            b, g, r = arr[:, :, 0], arr[:, :, 1], arr[:, :, 2]
        else:  # rgb
            r, g, b = arr[:, :, 0], arr[:, :, 1], arr[:, :, 2]
        return (0.299 * r + 0.587 * g + 0.114 * b).astype(np.uint8)
    raise SystemExit("encoding não suportado: {0}".format(enc))


def mk_header(frame_id, ns, seq):
    return Header(seq=seq, stamp=Time(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000)),
                  frame_id=frame_id)


def stamp_ns(h):
    return int(h.stamp.sec) * 1_000_000_000 + int(h.stamp.nanosec)


def main():
    ap = argparse.ArgumentParser(description="Converte bag ROS 2 -> ROS 1 (mono8) para o Kalibr.")
    ap.add_argument("bag")
    ap.add_argument("-o", "--out", required=True, help="Bag ROS 1 de saída (.bag).")
    ap.add_argument("--image-topics", nargs="+", required=True)
    ap.add_argument("--imu-topics", nargs="+", default=[])
    ap.add_argument("--limit", type=int, default=0, help="Máx. de mensagens por tópico (teste).")
    ap.add_argument("--keep-abs-time", action="store_true",
                    help="Não rebasear os timestamps (por padrão subtrai t0 ~ início do bag; necessário p/ o Kalibr).")
    args = ap.parse_args()

    bagdir = resolve_bag(args.bag)
    wanted = set(args.image_topics) | set(args.imu_topics)
    out = Path(args.out)
    if out.exists():
        out.unlink()

    counts = {t: 0 for t in wanted}
    seqs = {t: 0 for t in wanted}
    skipped = {}

    with AnyReader([bagdir]) as reader, Writer(out) as writer:
        t0 = 0 if args.keep_abs_time else int(reader.start_time) - 2_000_000_000
        print("T0_NS={0}   (rebasear tempo: {1}) -- use o MESMO --t0-ns no ros2_dvl_to_csv.py"
              .format(t0, not args.keep_abs_time))
        conns_in = [c for c in reader.connections if c.topic in wanted]
        found = {c.topic for c in conns_in}
        missing = wanted - found
        if missing:
            print("AVISO: tópicos não encontrados no bag:", missing)
        conns_out = {}
        for t in sorted(found):
            mt = "sensor_msgs/msg/Image" if t in args.image_topics else "sensor_msgs/msg/Imu"
            conns_out[t] = writer.add_connection(t, mt, typestore=TS1)

        for conn, _bagt, raw in reader.messages(connections=conns_in):
            t = conn.topic
            if args.limit and counts[t] >= args.limit:
                continue
            msg = reader.deserialize(raw, conn.msgtype)
            tns = stamp_ns(msg.header) - t0
            if tns < 0:
                tns = 0
            hdr = mk_header(msg.header.frame_id, tns, seqs[t])
            if t in args.image_topics:
                # O ZED publica ocasionalmente um frame vazio (visto 1x em 3325 no bag
                # 17-15-26 do v3). Escrever isso quebra o extrator do Kalibr.
                if msg.width == 0 or msg.height == 0 or len(msg.data) == 0:
                    skipped[t] = skipped.get(t, 0) + 1
                    continue
                gray = to_mono8(msg)
                out_msg = Image(header=hdr,
                                height=msg.height, width=msg.width, encoding="mono8",
                                is_bigendian=0, step=msg.width, data=gray.reshape(-1))
                writer.write(conns_out[t], tns, TS1.serialize_ros1(out_msg, "sensor_msgs/msg/Image"))
            else:
                out_msg = Imu(header=hdr,
                              orientation=Quaternion(x=msg.orientation.x, y=msg.orientation.y,
                                                     z=msg.orientation.z, w=msg.orientation.w),
                              orientation_covariance=np.asarray(msg.orientation_covariance, dtype=np.float64),
                              angular_velocity=Vector3(x=msg.angular_velocity.x, y=msg.angular_velocity.y,
                                                       z=msg.angular_velocity.z),
                              angular_velocity_covariance=np.asarray(msg.angular_velocity_covariance, dtype=np.float64),
                              linear_acceleration=Vector3(x=msg.linear_acceleration.x, y=msg.linear_acceleration.y,
                                                          z=msg.linear_acceleration.z),
                              linear_acceleration_covariance=np.asarray(msg.linear_acceleration_covariance, dtype=np.float64))
                writer.write(conns_out[t], tns, TS1.serialize_ros1(out_msg, "sensor_msgs/msg/Imu"))
            counts[t] += 1
            seqs[t] += 1

    print("Escrito {0}".format(out))
    for t in sorted(counts):
        print("  {0}: {1} msgs{2}".format(t, counts[t],
              "  (pulados {0} frames vazios)".format(skipped[t]) if skipped.get(t) else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
