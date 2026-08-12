#!/usr/bin/env python3
"""Extrai o stream do DVL (dvl_msgs/DVL) de um bag ROS 2 (MCAP/sqlite3) para o CSV do kalibr_calibrate_dvl.

Roda no HOST, sem ROS 2 instalado (usa a lib `rosbags`). Aceita um diretorio de bag rosbag2, um .mcap
com o metadata.yaml ao lado, ou um .zip (descompacta). Registra os tipos custom dvl_msgs/DVL e DVLBeam.

Saida: CSV de 16 colunas (ver scripts/config/dvl0_example.csv):
  timestamp_ns, vx, vy, vz, cov00..cov22 (9), velocity_valid, fom, altitude

Uso:
  pip install rosbags
  python3 ros2_dvl_to_csv.py <bag.zip|bag_dir|bag.mcap> -o dvl0.csv
  python3 ros2_dvl_to_csv.py <bag> --list-topics          # so lista os topicos e sai
"""
from __future__ import print_function
import argparse
import csv
import os
import sys
import zipfile
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

# --- definicoes das mensagens custom do DVL (github.com/paagutie/dvl_msgs) ---
DVLBEAM_MSG = """
int64 id
float64 velocity
float64 distance
float64 rssi
float64 nsd
bool valid
"""

DVL_MSG = """
std_msgs/Header header
float64 time
geometry_msgs/Vector3 velocity
float64 fom
float64[] covariance
float64 altitude
DVLBeam[] beams
bool velocity_valid
int64 status
int64 time_of_validity
int64 time_of_transmission
string form
"""

CSV_HEADER = ["timestamp_ns", "vx", "vy", "vz",
              "cov00", "cov01", "cov02", "cov10", "cov11", "cov12", "cov20", "cov21", "cov22",
              "velocity_valid", "fom", "altitude"]


def make_typestore():
    ts = get_typestore(Stores.ROS2_HUMBLE)
    types = {}
    types.update(get_types_from_msg(DVLBEAM_MSG, "dvl_msgs/msg/DVLBeam"))
    types.update(get_types_from_msg(DVL_MSG, "dvl_msgs/msg/DVL"))
    ts.register(types)
    return ts


def resolve_bag_path(path):
    """Retorna um diretorio de bag rosbag2 (com metadata.yaml). Descompacta .zip se necessario."""
    p = Path(path)
    if p.is_dir():
        return p
    if p.suffix == ".zip":
        outdir = p.with_suffix("")  # <dir>/<stem>/
        if outdir.is_dir() and (outdir / "metadata.yaml").exists():
            print("Já descompactado em: {0}".format(outdir))
            return outdir
        print("Descompactando {0} -> {1} (pode demorar / usar bastante disco)...".format(p, outdir))
        outdir.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(p) as zf:
            zf.extractall(outdir)
        # alguns zips tem tudo dentro de uma subpasta; localiza o metadata.yaml
        if not (outdir / "metadata.yaml").exists():
            for md in outdir.rglob("metadata.yaml"):
                return md.parent
        return outdir
    # .mcap solto: AnyReader precisa do metadata.yaml ao lado -> usa a pasta pai
    if p.suffix == ".mcap":
        return p.parent
    raise SystemExit("Caminho de bag nao reconhecido: {0}".format(path))


def list_topics(bagdir, typestore):
    with AnyReader([Path(bagdir)], default_typestore=typestore) as reader:
        print("Tópicos no bag ({0} msgs, {1:.1f}s):".format(
            reader.message_count, reader.duration / 1e9))
        rows = sorted(((c.topic, c.msgtype, c.msgcount) for c in reader.connections),
                      key=lambda r: r[0])
        for topic, msgtype, count in rows:
            print("  {0:45s} {1:35s} {2}".format(topic, msgtype, count))


def extract(bagdir, dvl_topic, out_csv, timestamp_source, typestore, t0_ns=0):
    n = 0
    with AnyReader([Path(bagdir)], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections if c.topic == dvl_topic]
        if not conns:
            raise SystemExit("Tópico do DVL '{0}' não encontrado. Use --list-topics para ver os tópicos."
                             .format(dvl_topic))
        with open(out_csv, "w", newline="") as f:
            w = csv.writer(f)
            f.write("# " + ",".join(CSV_HEADER) + "\n")
            for conn, bagtime, rawdata in reader.messages(connections=conns):
                msg = reader.deserialize(rawdata, conn.msgtype)
                # timestamp
                hs = msg.header.stamp
                header_ns = int(hs.sec) * 1_000_000_000 + int(hs.nanosec)
                if timestamp_source == "bag" or header_ns == 0:
                    ts_ns = int(bagtime)
                else:
                    ts_ns = header_ns
                ts_ns = ts_ns - t0_ns   # rebasear (mesmo t0 do bag ROS 1; ver D8)
                # velocidade
                v = msg.velocity
                # covariancia (flat; pad/trunca para 9). Pode ser numpy array -> nao usar 'or'.
                cov_raw = getattr(msg, "covariance", None)
                cov = list(cov_raw) if cov_raw is not None else []
                cov = (cov + [0.0] * 9)[:9]
                valid = 1 if bool(msg.velocity_valid) else 0
                row = [ts_ns, float(v.x), float(v.y), float(v.z)] + [float(c) for c in cov] + \
                      [valid, float(msg.fom), float(msg.altitude)]
                w.writerow(row)
                n += 1
    print("Escrito {0} amostras de DVL em {1}".format(n, out_csv))
    return n


def main():
    ap = argparse.ArgumentParser(description="Extrai dvl_msgs/DVL de um bag ROS 2 para CSV.")
    ap.add_argument("bag", help="Diretório do bag rosbag2, .mcap (com metadata.yaml ao lado) ou .zip.")
    ap.add_argument("--dvl-topic", default="/dvl/data", help="Tópico do DVL (default: %(default)s).")
    ap.add_argument("-o", "--out", default="dvl0.csv", help="CSV de saída (default: %(default)s).")
    ap.add_argument("--timestamp", choices=["header", "bag"], default="header",
                    help="Fonte do timestamp: header.stamp (default) ou tempo de gravação do bag.")
    ap.add_argument("--list-topics", action="store_true", help="Apenas listar os tópicos e sair.")
    ap.add_argument("--t0-ns", type=int, default=0,
                    help="Subtrai este t0 (ns) do timestamp — use o MESMO valor T0_NS do ros2_to_ros1_kalibr.py (D8).")
    args = ap.parse_args()

    typestore = make_typestore()
    bagdir = resolve_bag_path(args.bag)

    if args.list_topics:
        list_topics(bagdir, typestore)
        return 0

    extract(bagdir, args.dvl_topic, args.out, args.timestamp, typestore, t0_ns=args.t0_ns)
    return 0


if __name__ == "__main__":
    sys.exit(main())
