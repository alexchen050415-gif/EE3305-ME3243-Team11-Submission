"""从ROS2实际记录的CSV生成单次和双项目对比图。没有假数据。"""
import csv
import base64
import zlib
import json
from pathlib import Path

MODES = ("Original", "Improved")
DISPLAY = {"Original":"Original | Dijkstra + Fixed PP",
           "Improved":"Improved | Refined Dijkstra + Fixed PP"}


def update_plots(folder):
    folder=Path(folder)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    records=json.loads((folder/"records.json").read_text(encoding="utf-8"))
    chosen={r["variant"]:r for r in records if r.get("variant") in MODES}
    datasets={}
    for mode,row in chosen.items():
        with (folder/row["csv"]).open(newline="",encoding="utf-8") as f:
            samples=[{k:float(v) for k,v in item.items()} for item in csv.DictReader(f)]
        if samples:datasets[mode]=(row,samples)
    if not datasets:return "no samples"

    #图一：整张原始地图的障碍物点云，加上两套机器人实际轨迹
    fig,ax=plt.subplots(figsize=(11.6,8.5),dpi=145)
    mapfile=folder/"map_occupancy.json"
    if mapfile.exists():
        import numpy as np
        saved=json.loads(mapfile.read_text(encoding="utf-8"))
        w,h=int(saved["width"]),int(saved["height"])
        res=float(saved["resolution"])
        ox,oy=float(saved["origin_x"]),float(saved["origin_y"])
        yaw=float(saved.get("yaw",0.0))
        data=np.frombuffer(zlib.decompress(base64.b64decode(saved["data"])),
                           dtype=np.uint8)
        if data.size!=w*h:
            raise ValueError("Saved occupancy grid size mismatch")
        occ=data.reshape(h,w).astype(np.int16)-1
        def dots(mask,size,color,label,alpha):
            rr,cc=np.nonzero(mask)
            if rr.size==0:return
            u=(cc+0.5)*res;v=(rr+0.5)*res
            x=ox+u*np.cos(yaw)-v*np.sin(yaw)
            y=oy+u*np.sin(yaw)+v*np.cos(yaw)
            ax.scatter(x,y,s=size,c=color,marker="s",lw=0,alpha=alpha,
                       label=label,rasterized=True,zorder=1)
        #灰点是地图未知/未确认区域；黑点才是原始地图里的真实占用单元
        dots((occ<0)|((occ>0)&(occ<65)),2.8,"#c3cbd3","Unknown / unconfirmed",0.5)
        dots(occ>=65,3.6,"#293443","Occupied cells (raw /map)",1.0)
        corners=np.array([[0,0],[w*res,0],[w*res,h*res],[0,h*res]])
        xx=ox+corners[:,0]*np.cos(yaw)-corners[:,1]*np.sin(yaw)
        yy=oy+corners[:,0]*np.sin(yaw)+corners[:,1]*np.cos(yaw)
        ax.set_xlim(min(xx),max(xx))
        ax.set_ylim(min(yy),max(yy))
    else:
        ax.text(0.5,0.98,"Raw /map not received for this run; obstacle dots unavailable",
                transform=ax.transAxes,ha="center",va="top",fontsize=9)
    for mode in MODES:
        if mode not in datasets:continue
        meta,samples=datasets[mode]
        line,=ax.plot([s["x"] for s in samples],[s["y"] for s in samples],
                       lw=2.2,label=f"{DISPLAY[mode]} | actual ({meta['status']})",zorder=5)
        path=meta.get("reference_path",[])
        if len(path)>=2:
            ax.plot([p[0] for p in path],[p[1] for p in path],ls="--",lw=1.1,
                    color=line.get_color(),alpha=.65,label=f"{mode} planned",zorder=4)
        #各自起点和终点可查看是否精确对齐
        sx,sy=meta["start"][:2]
        ax.scatter([sx],[sy],marker="o",s=38,facecolors="white",
                   edgecolors=line.get_color(),lw=1.7,zorder=7)
    first=next(iter(datasets.values()))[0]
    ax.scatter([first["goal"][0]],[first["goal"][1]],marker="*",s=155,
               color="#dd8528",edgecolors="white",linewidths=.6,label="Selected goal",zorder=8)
    ax.set(title="Actual trajectories over complete raw occupancy map",
           xlabel="X (m)",ylabel="Y (m)")
    ax.set_aspect("equal",adjustable="box")
    ax.grid(alpha=.16)
    #把图例放到地图下面，避免挡住墙角、障碍点和轨迹
    ax.legend(fontsize=8,loc="upper center",bbox_to_anchor=(.5,-.12),ncol=2,frameon=False)
    fig.tight_layout();fig.savefig(folder/"trajectory_comparison.png",bbox_inches="tight",pad_inches=.16);plt.close(fig)

    #图二：里程计的真实线速度，按每次开始运动起计时
    fig,ax=plt.subplots(figsize=(9,5),dpi=135)
    for mode in MODES:
        if mode not in datasets:continue
        _,samples=datasets[mode]
        ax.plot([s["t"] for s in samples],[s["v"] for s in samples],lw=1.7,label=DISPLAY[mode])
    ax.set(title="Actual linear speed (odom)",xlabel="Time since goal (s)",ylabel="Speed (m/s)")
    ax.grid(alpha=.25);ax.legend(fontsize=8);fig.tight_layout()
    fig.savefig(folder/"speed_comparison.png");plt.close(fig)

    #图三：同一条行进路程上离障碍物多近（网格近似，不是碰撞证明）
    fig,ax=plt.subplots(figsize=(9,5),dpi=135)
    for mode in MODES:
        if mode not in datasets:continue
        _,samples=datasets[mode]
        ax.plot([s["distance"] for s in samples],
                [s["clearance"] for s in samples],lw=1.7,label=DISPLAY[mode])
    ax.axhline(0,linestyle="--",color="gray",lw=1,label="Estimated contact boundary")
    ax.set(title="Approx. obstacle clearance (grid map)",
           xlabel="Travelled distance (m)",ylabel="Clearance after robot radius (m)")
    ax.grid(alpha=.25);ax.legend(fontsize=8);fig.tight_layout()
    fig.savefig(folder/"clearance_comparison.png");plt.close(fig)

    #图四：区分控制器发的指令和Gazebo实际速度
    fig,ax=plt.subplots(figsize=(9,5),dpi=135)
    for mode in MODES:
        if mode not in datasets:continue
        _,samples=datasets[mode]
        line,=ax.plot([s["t"] for s in samples],[s["cmd_v"] for s in samples],
                       lw=1.5,label=f"{mode} commanded")
        ax.plot([s["t"] for s in samples],[s["v"] for s in samples],
                ls="--",lw=1,color=line.get_color(),label=f"{mode} actual")
    ax.set(title="Command vs actual velocity",xlabel="Time since goal (s)",ylabel="Speed (m/s)")
    ax.grid(alpha=.25);ax.legend(fontsize=8);fig.tight_layout()
    fig.savefig(folder/"command_vs_actual.png");plt.close(fig)

    fields=["variant","status","elapsed_s","distance_m","goal_error_m","min_clearance_m","csv"]
    with (folder/"summary.csv").open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=fields)
        w.writeheader()
        for mode in MODES:
            if mode in chosen:w.writerow({k:chosen[mode].get(k) for k in fields})
    return "four figures + summary.csv saved" 
