#!/usr/bin/env python3
import argparse, csv, json, math, statistics, platform, subprocess, sys
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageOps, ImageDraw
from openpyxl import Workbook

COND43 = [
 ("condition_01","Kondisi Aman Simetris — GREEN | GREEN"),
 ("condition_02","Peringatan Sisi Kiri — YELLOW | GREEN"),
 ("condition_03","Kondisi Kritis Sisi Kiri — RED | GREEN"),
 ("condition_04","Peringatan Sisi Kanan — GREEN | YELLOW"),
 ("condition_05","Kondisi Kritis Sisi Kanan — GREEN | RED"),
 ("condition_06","Area Tidak Dapat Dilalui — GRAY | GRAY"),
]
COND44=[("condition_A","Jalur Relatif Kosong"),("condition_B","Satu Manusia"),("condition_C","Kondisi Kompleks")]

def load_json(p):
    try: return json.loads(Path(p).read_text())
    except Exception: return {}

def finite(v):
    try:
        x=float(v); return x if math.isfinite(x) else None
    except Exception: return None

def stats(vals):
    a=[x for x in (finite(v) for v in vals) if x is not None]
    return {"N":len(a),"mean":statistics.fmean(a) if a else None,
            "std":statistics.stdev(a) if len(a)>1 else (0.0 if a else None),
            "min":min(a) if a else None,"max":max(a) if a else None}

def percentile(vals,p):
    a=sorted(x for x in (finite(v) for v in vals) if x is not None)
    if not a:return None
    if len(a)==1:return a[0]
    pos=(len(a)-1)*float(p)
    lo=int(math.floor(pos));hi=int(math.ceil(pos))
    if lo==hi:return a[lo]
    w=pos-lo
    return a[lo]*(1.0-w)+a[hi]*w
def sub(state,key):
    return state.get("subchapters",{}).get(key,{})
def conditions(state,key):
    return sub(state,key).get("conditions",{})
def rows_all(state,key):
    out=[]
    for c in conditions(state,key).values(): out += c.get("rows",[])
    return out

def write_csv(path, rows, fields):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    with path.open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=fields,extrasaction="ignore"); w.writeheader(); w.writerows(rows)

def write_xlsx(path, sheets):
    wb=Workbook(); first=True
    for name,rows in sheets:
        ws=wb.active if first else wb.create_sheet(); first=False; ws.title=str(name)[:31]
        fields=[]
        for r in rows:
            for k in r.keys():
                if k not in fields: fields.append(k)
        if not fields:
            ws.append(["status"]); ws.append(["belum ada data"]); continue
        ws.append(fields)
        for r in rows:
            vals=[]
            for k in fields:
                v=r.get(k)
                if isinstance(v,(dict,list)): v=json.dumps(v,ensure_ascii=False)
                vals.append(v)
            ws.append(vals)
        ws.freeze_panes="A2"; ws.auto_filter.ref=ws.dimensions
    Path(path).parent.mkdir(parents=True,exist_ok=True); wb.save(path)

def save_plot(path,title,xlabel,ylabel,x,y):
    fig=plt.figure(figsize=(9,5)); ax=fig.add_subplot(111)
    ax.plot(x,y,marker="o"); ax.set_title(title); ax.set_xlabel(xlabel); ax.set_ylabel(ylabel); ax.grid(True,alpha=.25)
    fig.tight_layout(); fig.savefig(path,dpi=160); plt.close(fig)

def save_bar(path,title,labels,values,ylabel):
    fig=plt.figure(figsize=(10,5)); ax=fig.add_subplot(111)
    ax.bar(labels,values); ax.set_title(title); ax.set_ylabel(ylabel); ax.tick_params(axis="x",rotation=20); ax.grid(True,axis="y",alpha=.25)
    fig.tight_layout(); fig.savefig(path,dpi=160); plt.close(fig)

def representative(rows, root):
    valid=[r for r in rows if r.get("frame_filename")]
    if not valid: return None
    best=max(valid,key=lambda r: finite(r.get("confidence")) or -1)
    p=root/best["frame_filename"]
    return p if p.is_file() else None
def montage(path,title,items,root,cols):
    def safe_text(s):
        return str(s).encode("latin-1","replace").decode("latin-1")
    imgs=[]
    for label,rows in items:
        p=representative(rows,root)
        if p:
            try: im=Image.open(p).convert("RGB")
            except Exception: im=Image.new("RGB",(640,360),"white")
        else: im=Image.new("RGB",(640,360),"white")
        im.thumbnail((520,300)); canvas=Image.new("RGB",(540,350),"white")
        canvas.paste(im,((540-im.width)//2,35)); ImageDraw.Draw(canvas).text((12,10),safe_text(label),fill="black")
        imgs.append(canvas)
    if not imgs:return
    rowsn=math.ceil(len(imgs)/cols); out=Image.new("RGB",(cols*540,60+rowsn*350),"white")
    ImageDraw.Draw(out).text((15,15),safe_text(title),fill="black")
    for i,im in enumerate(imgs):out.paste(im,((i%cols)*540,60+(i//cols)*350))
    out.save(path)

def export41(root,state):
    d=root/"4.1_VALIDASI_SISTEM"; d.mkdir(exist_ok=True)
    rows=rows_all(state,"4.1")
    summary={"samples":len(rows),"latest":rows[-1] if rows else None}
    (d/"validation_summary.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False))
    if rows:
        fps_ref=[finite(r.get("fps_reference_hz")) for r in rows];fps_ref=[x for x in fps_ref if x is not None]
        fps_act=[finite(r.get("fps_actual_hz")) for r in rows];fps_act=[x for x in fps_act if x is not None]
        fps_err=[finite(r.get("fps_error_hz")) for r in rows];fps_err=[x for x in fps_err if x is not None]
        live=sum(1 for r in rows if r.get("stream_status")=="LIVE")
        s41=[{
            "samples":len(rows),
            "fps_reference_hz":statistics.fmean(fps_ref) if fps_ref else None,
            "fps_actual_mean_hz":statistics.fmean(fps_act) if fps_act else None,
            "fps_bias_hz":statistics.fmean(fps_err) if fps_err else None,
            "fps_mae_hz":statistics.fmean(abs(x) for x in fps_err) if fps_err else None,
            "fps_rmse_hz":math.sqrt(statistics.fmean(x*x for x in fps_err)) if fps_err else None,
            "fps_p95_abs_error_hz":percentile([abs(x) for x in fps_err],.95),
            "stream_availability_pct":100.0*live/len(rows) if rows else None
        }]
        write_csv(d/"summary.csv",s41,list(s41[0].keys()))
    if rows:
        fields=[]
        for r in rows:
            for k in r:
                if k not in fields:fields.append(k)
        write_csv(d/"raw_data.csv",rows,fields)
        write_xlsx(d/"raw_data.xlsx",[("validation",rows)])
    meta=load_json(root/"metadata.json")
    cfg=meta.get("config_snapshot") or {}
    (d/"config_snapshot.json").write_text(json.dumps(cfg,indent=2,ensure_ascii=False))
    cfg_rows=[]
    if isinstance(cfg,dict):
        for k in sorted(cfg):
            v=cfg[k]
            cfg_rows.append({"key":k,"value":json.dumps(v,ensure_ascii=False) if isinstance(v,(dict,list)) else v})
    write_csv(d/"config_summary.csv",cfg_rows,["key","value"])
    u=platform.uname()
    (d/"system_info.txt").write_text(
        "system="+u.system+"\nnode="+u.node+"\nrelease="+u.release+"\nversion="+u.version+
        "\nmachine="+u.machine+"\npython="+sys.version.replace("\n"," ")+"\n")
    try:
        git=subprocess.run(["git","-C",str(root.parents[2]),"rev-parse","HEAD"],capture_output=True,text=True,timeout=5)
        commit=git.stdout.strip() if git.returncode==0 else "UNAVAILABLE"
    except Exception:
        commit="UNAVAILABLE"
    (d/"git_commit.txt").write_text(commit+"\n")
    if rows:
        x=list(range(1,len(rows)+1))
        save_plot(d/"fps_graph.png","FPS Aktual Sistem Persepsi","Sampel","FPS",x,[finite(r.get("fps")) or 0 for r in rows])
        save_plot(d/"frame_age_graph.png","Umur Frame","Sampel","Frame Age [ms]",x,[finite(r.get("frame_age_ms")) or 0 for r in rows])
        fig=plt.figure(figsize=(9,5));ax=fig.add_subplot(111)
        ax.plot(x,[finite(r.get("fps_reference_hz")) or 0 for r in rows],linestyle="--",label="Reference / target")
        ax.plot(x,[finite(r.get("fps_actual_hz")) or 0 for r in rows],marker="o",label="Actual")
        ax.set_title("Target vs Aktual Pipeline FPS");ax.set_xlabel("Sampel");ax.set_ylabel("FPS");ax.grid(True,alpha=.25);ax.legend()
        fig.tight_layout();fig.savefig(d/"fps_reference_vs_actual.png",dpi=160);plt.close(fig)
        write_csv(d/"graph_source_manifest.csv",[
            {"graph":"fps_graph.png","raw_source":"raw_data.csv","columns":"capture_id,timestamp,fps"},
            {"graph":"fps_reference_vs_actual.png","raw_source":"raw_data.csv","columns":"capture_id,fps_reference_hz,fps_actual_hz,fps_error_hz"},
            {"graph":"frame_age_graph.png","raw_source":"raw_data.csv","columns":"capture_id,timestamp,frame_age_ms"}],
            ["graph","raw_source","columns"])
def export42(root,state):
    d=root/"4.2_HUMAN_DETECTION"; d.mkdir(exist_ok=True)
    lights=[("night","MALAM / minim cahaya"),("day","SIANG / banyak cahaya")]
    cs=conditions(state,"4.2")

    def rows42(light,m):
        key=f"human_{light}_{m}m"
        if key in cs:
            rr=cs.get(key,{}).get("rows",[])
        elif light=="night":
            rr=cs.get(f"human_{m}m",{}).get("rows",[])
        else:
            rr=[]
        out=[]
        for row in rr:
            x=dict(row)
            x.setdefault("lighting_condition",light)
            x.setdefault("illumination_label","SIANG_BANYAK_CAHAYA" if light=="day" else "MALAM_MINIM_CAHAYA")
            out.append(x)
        return out

    def frac(row,key,size_key,image_key):
        value=finite(row.get(key))
        if value is not None:return value
        size,image=finite(row.get(size_key)),finite(row.get(image_key))
        return size/image if size is not None and image is not None and image>0 else None

    summary=[];items=[];all_rows=[];sheets=[]
    by_light={light:{} for light,_ in lights}
    for light,label in lights:
        for m in range(1,6):
            rr=rows42(light,m);by_light[light][m]=rr;all_rows.extend(rr);sheets.append((f"{light}_{m}m",rr))
            items.append((f"{label} • {m} meter",rr))
            det=[r for r in rr if int(r.get("detected") or 0)==1]
            conf=[finite(r.get("confidence")) for r in det];conf=[x for x in conf if x is not None]
            err=[finite(r.get("distance_error_m")) for r in rr];err=[x for x in err if x is not None]
            abserr=[abs(x) for x in err]
            est=[finite(r.get("estimated_distance_m")) for r in rr];est=[x for x in est if x is not None]
            luma=[finite(r.get("illumination_mean_luma")) for r in rr];luma=[x for x in luma if x is not None]
            luma_std=[finite(r.get("illumination_stddev_luma")) for r in rr];luma_std=[x for x in luma_std if x is not None]
            hfrac=[frac(r,"bbox_height_fraction","bbox_height","image_height") for r in det];hfrac=[x for x in hfrac if x is not None]
            wfrac=[frac(r,"bbox_width_fraction","bbox_width","image_width") for r in det];wfrac=[x for x in wfrac if x is not None]
            saturated=[r for r in det if r.get("bbox_saturated") is True or str(r.get("distance_quality") or "")=="SATURATED_NEAR"
                       or (frac(r,"bbox_height_fraction","bbox_height","image_height") or 0)>=0.95]
            valid=[r for r in det if r not in saturated and finite(r.get("distance_error_m")) is not None]
            valid_err=[finite(r.get("distance_error_m")) for r in valid]
            row={"lighting_condition":light,"lighting_label":label,"distance_m":m,"N":len(rr),
                 "detected":len(det),"detection_rate_pct":100*len(det)/len(rr) if rr else 0,
                 "confidence_mean":statistics.fmean(conf) if conf else None,
                 "confidence_std":statistics.stdev(conf) if len(conf)>1 else (0 if conf else None),
                 "confidence_min":min(conf) if conf else None,"confidence_max":max(conf) if conf else None,
                 "illumination_mean_luma":statistics.fmean(luma) if luma else None,
                 "illumination_std_luma_mean":statistics.fmean(luma_std) if luma_std else None,
                 "estimated_distance_mean_m":statistics.fmean(est) if est else None,
                 "estimated_distance_std_m":statistics.stdev(est) if len(est)>1 else (0 if est else None),
                 "distance_bias_m":statistics.fmean(err) if err else None,
                 "mae_distance_m":statistics.fmean(abserr) if abserr else None,
                 "rmse_distance_m":math.sqrt(statistics.fmean(x*x for x in err)) if err else None,
                 "median_abs_error_m":statistics.median(abserr) if abserr else None,
                 "p95_abs_error_m":percentile(abserr,.95),
                 "bbox_height_fraction_mean":statistics.fmean(hfrac) if hfrac else None,
                 "bbox_width_fraction_mean":statistics.fmean(wfrac) if wfrac else None,
                 "bbox_saturated_N":len(saturated),
                 "bbox_saturated_pct":100*len(saturated)/len(det) if det else 0,
                 "calibration_valid_N":len(valid),
                 "mae_valid_bbox_m":statistics.fmean(abs(x) for x in valid_err) if valid_err else None,
                 "rmse_valid_bbox_m":math.sqrt(statistics.fmean(x*x for x in valid_err)) if valid_err else None}
            summary.append(row)

    fields=[]
    for r in all_rows:
        for k in r:
            if k not in fields:fields.append(k)
    if fields:
        write_csv(d/"raw_all.csv",all_rows,fields)
        for light,_ in lights:
            lr=[r for r in all_rows if r.get("lighting_condition")==light]
            write_csv(d/f"raw_{light}.csv",lr,fields)
    write_csv(d/"summary.csv",summary,list(summary[0].keys()))
    write_xlsx(d/"raw_data.xlsx",sheets+[("summary",summary)])

    # Calibration input excludes no detections but intentionally keeps saturation
    # flags so the later fit can choose near/far features without hiding data.
    cal_fields=["lighting_condition","reference_distance_m","confidence",
                "illumination_mean_luma","bbox_width_fraction","bbox_height_fraction",
                "stable_bbox_height_fraction","bottom_clipped","bbox_saturated",
                "raw_forward_m","bbox_calibrated_forward_m","calibrated_forward_m",
                "stable_forward_m","nav2_forward_m","estimated_distance_m","distance_error_m"]
    write_csv(d/"calibration_candidate_input.csv",all_rows,cal_fields)

    # Candidate distance models are ANALYSIS ONLY. They are cross-light validated
    # and are never written back to runtime YAML automatically.
    def model_samples(light):
        out=[]
        for row in all_rows:
            if row.get("lighting_condition")!=light or int(row.get("detected") or 0)!=1:
                continue
            ref=finite(row.get("reference_distance_m"))
            hf=frac(row,"bbox_height_fraction","bbox_height","image_height")
            wf=frac(row,"bbox_width_fraction","bbox_width","image_width")
            if ref is None or hf is None or wf is None or wf<=0 or hf<=0:
                continue
            out.append((ref,hf,wf))
        return out

    def fit_eval(train,test,model_name):
        if model_name=="height_linear":
            feature=lambda x:[1.0,x[1]]
        elif model_name=="inverse_width":
            feature=lambda x:[1.0,1.0/x[2]]
        else:
            feature=lambda x:[1.0,1.0/x[2],x[1]]
        if len(train)<5 or len(test)<5:
            return None
        X=np.asarray([feature(x) for x in train],dtype=float)
        y=np.asarray([x[0] for x in train],dtype=float)
        Xt=np.asarray([feature(x) for x in test],dtype=float)
        yt=np.asarray([x[0] for x in test],dtype=float)
        try:
            coef=np.linalg.lstsq(X,y,rcond=None)[0]
        except Exception:
            return None
        pred=Xt@coef
        err=pred-yt
        return {
            "model":model_name,
            "coefficients":";".join(f"{float(v):.10g}" for v in coef),
            "n_train":len(train),"n_test":len(test),
            "bias_m":float(np.mean(err)),
            "mae_m":float(np.mean(np.abs(err))),
            "rmse_m":float(np.sqrt(np.mean(err*err))),
            "p95_abs_error_m":float(np.percentile(np.abs(err),95))
        }

    model_rows=[]
    samples={light:model_samples(light) for light,_ in lights}
    for train_light,test_light in (("day","night"),("night","day")):
        for model_name in ("height_linear","inverse_width","combined_invwidth_height"):
            r=fit_eval(samples[train_light],samples[test_light],model_name)
            if r:
                r["train_lighting"]=train_light;r["test_lighting"]=test_light
                model_rows.append(r)
    if model_rows:
        write_csv(d/"distance_model_comparison.csv",model_rows,
                  ["model","train_lighting","test_lighting","coefficients","n_train","n_test",
                   "bias_m","mae_m","rmse_m","p95_abs_error_m"])
        aggregate=[]
        for model_name in sorted({r["model"] for r in model_rows}):
            rr=[r for r in model_rows if r["model"]==model_name]
            aggregate.append({
                "model":model_name,
                "cross_light_runs":len(rr),
                "mean_cross_light_mae_m":statistics.fmean(r["mae_m"] for r in rr),
                "mean_cross_light_rmse_m":statistics.fmean(r["rmse_m"] for r in rr),
                "worst_cross_light_rmse_m":max(r["rmse_m"] for r in rr),
                "status":"ANALYSIS_ONLY_NOT_DEPLOYED"
            })
        write_csv(d/"distance_model_crosslight_summary.csv",aggregate,list(aggregate[0].keys()))
    else:
        write_csv(d/"distance_model_comparison.csv",[],[
            "model","train_lighting","test_lighting","coefficients","n_train","n_test",
            "bias_m","mae_m","rmse_m","p95_abs_error_m"])

    # Aggregate lighting effect on detector score (descriptive, not a distance model).
    lighting_rows=[]
    for light,label in lights:
        rr=[r for r in all_rows if r.get("lighting_condition")==light]
        detected=[r for r in rr if int(r.get("detected") or 0)==1]
        conf=[finite(r.get("confidence")) for r in detected];conf=[x for x in conf if x is not None]
        luma=[finite(r.get("illumination_mean_luma")) for r in rr];luma=[x for x in luma if x is not None]
        lighting_rows.append({
            "lighting_condition":light,"lighting_label":label,"N":len(rr),
            "detection_rate_pct":100*len(detected)/len(rr) if rr else 0,
            "confidence_mean":statistics.fmean(conf) if conf else None,
            "confidence_std":statistics.stdev(conf) if len(conf)>1 else (0 if conf else None),
            "mean_luma":statistics.fmean(luma) if luma else None
        })
    write_csv(d/"lighting_comparison.csv",lighting_rows,list(lighting_rows[0].keys()))

    distances=list(range(1,6))
    def metric(light,key):
        return [next((finite(r.get(key)) for r in summary
                      if r["lighting_condition"]==light and r["distance_m"]==m),None) or 0
                for m in distances]

    # Confidence SIANG vs MALAM.
    fig=plt.figure(figsize=(9,5));ax=fig.add_subplot(111)
    for light,label in lights:ax.plot(distances,metric(light,"confidence_mean"),marker="o",label=label)
    ax.set_title("Confidence Deteksi Manusia: SIANG vs MALAM");ax.set_xlabel("Jarak Referensi [m]");ax.set_ylabel("Confidence")
    ax.set_xticks(distances);ax.grid(True,alpha=.25);ax.legend();fig.tight_layout();fig.savefig(d/"confidence_vs_distance.png",dpi=160);plt.close(fig)

    # Detection rate SIANG vs MALAM.
    fig=plt.figure(figsize=(9,5));ax=fig.add_subplot(111);x=list(range(len(distances)));w=.36
    ax.bar([i-w/2 for i in x],metric("night","detection_rate_pct"),w,label="MALAM")
    ax.bar([i+w/2 for i in x],metric("day","detection_rate_pct"),w,label="SIANG")
    ax.set_xticks(x,[f"{m} m" for m in distances]);ax.set_ylabel("Detection Rate [%]");ax.set_title("Detection Rate: SIANG vs MALAM")
    ax.grid(True,axis="y",alpha=.25);ax.legend();fig.tight_layout();fig.savefig(d/"detection_rate.png",dpi=160);plt.close(fig)

    # Reference vs actual/estimated distance with ideal y=x.
    fig=plt.figure(figsize=(9,5));ax=fig.add_subplot(111)
    ax.plot(distances,distances,linestyle="--",label="Ideal y=x")
    for light,label in lights:ax.plot(distances,metric(light,"estimated_distance_mean_m"),marker="o",label=label)
    ax.set_title("Jarak Referensi vs Estimasi: SIANG vs MALAM");ax.set_xlabel("Jarak Referensi [m]");ax.set_ylabel("Rata-rata Jarak Estimasi [m]")
    ax.set_xticks(distances);ax.grid(True,alpha=.25);ax.legend();fig.tight_layout();fig.savefig(d/"estimated_vs_reference.png",dpi=160);plt.close(fig)

    # MAE and P95 absolute error.
    fig=plt.figure(figsize=(9,5));ax=fig.add_subplot(111);x=list(range(len(distances)));w=.36
    ax.bar([i-w/2 for i in x],metric("night","mae_distance_m"),w,label="MAE MALAM")
    ax.bar([i+w/2 for i in x],metric("day","mae_distance_m"),w,label="MAE SIANG")
    ax.set_xticks(x,[f"{m} m" for m in distances]);ax.set_ylabel("MAE [m]");ax.set_title("MAE Estimasi Jarak: SIANG vs MALAM")
    ax.grid(True,axis="y",alpha=.25);ax.legend();fig.tight_layout();fig.savefig(d/"distance_error.png",dpi=160);plt.close(fig)

    fig=plt.figure(figsize=(9,5));ax=fig.add_subplot(111)
    for light,label in lights:ax.plot(distances,metric(light,"p95_abs_error_m"),marker="o",label=label)
    ax.set_title("P95 Absolute Error Jarak: SIANG vs MALAM");ax.set_xlabel("Jarak Referensi [m]");ax.set_ylabel("P95 |error| [m]")
    ax.set_xticks(distances);ax.grid(True,alpha=.25);ax.legend();fig.tight_layout();fig.savefig(d/"distance_error_p95.png",dpi=160);plt.close(fig)

    # BBox scale and illumination evidence.
    for key,name,ylabel in [
        ("bbox_height_fraction_mean","bbox_height_fraction_vs_distance.png","BBox Height / Image Height"),
        ("bbox_width_fraction_mean","bbox_width_fraction_vs_distance.png","BBox Width / Image Width"),
        ("illumination_mean_luma","illumination_luma_vs_distance.png","Mean Luma")]:
        fig=plt.figure(figsize=(9,5));ax=fig.add_subplot(111)
        for light,label in lights:ax.plot(distances,metric(light,key),marker="o",label=label)
        ax.set_title(("Pencahayaan Aktual" if key=="illumination_mean_luma" else "Skala BBox")+" : SIANG vs MALAM")
        ax.set_xlabel("Jarak Referensi [m]");ax.set_ylabel(ylabel);ax.set_xticks(distances);ax.grid(True,alpha=.25);ax.legend()
        fig.tight_layout();fig.savefig(d/name,dpi=160);plt.close(fig)

    montage(d/"composite_10_condition.png","Dokumentasi Human 1–5 m • MALAM dan SIANG",items,root,5)

    lineage=[
      {"graph":"confidence_vs_distance.png","raw_source":"raw_all.csv","columns":"lighting_condition,reference_distance_m,detected,confidence"},
      {"graph":"detection_rate.png","raw_source":"raw_all.csv","columns":"lighting_condition,reference_distance_m,detected"},
      {"graph":"estimated_vs_reference.png","raw_source":"raw_all.csv","columns":"lighting_condition,reference_distance_m,estimated_distance_m,distance_error_m,abs_distance_error_m"},
      {"graph":"distance_error.png","raw_source":"raw_all.csv","columns":"lighting_condition,reference_distance_m,estimated_distance_m,distance_error_m,abs_distance_error_m"},
      {"graph":"distance_error_p95.png","raw_source":"raw_all.csv","columns":"lighting_condition,reference_distance_m,distance_error_m"},
      {"graph":"bbox_height_fraction_vs_distance.png","raw_source":"raw_all.csv","columns":"lighting_condition,reference_distance_m,bbox_height_fraction,bbox_height,image_height"},
      {"graph":"bbox_width_fraction_vs_distance.png","raw_source":"raw_all.csv","columns":"lighting_condition,reference_distance_m,bbox_width_fraction,bbox_width,image_width"},
      {"graph":"illumination_luma_vs_distance.png","raw_source":"raw_all.csv","columns":"lighting_condition,reference_distance_m,illumination_mean_luma,illumination_stddev_luma"},
      {"graph":"distance_model_comparison.csv","raw_source":"calibration_candidate_input.csv","columns":"lighting_condition,reference_distance_m,bbox_width_fraction,bbox_height_fraction"},
      {"graph":"lighting_comparison.csv","raw_source":"raw_all.csv","columns":"lighting_condition,detected,confidence,illumination_mean_luma"}
    ]
    write_csv(d/"graph_source_manifest.csv",lineage,["graph","raw_source","columns"])
    (d/"calibration_protocol.txt").write_text(
        "4.2 DISTANCE RECALIBRATION PROTOCOL\n"
        "- Runtime coefficients are NOT auto-changed by this export.\n"
        "- Collect 10 unique captures at each 1..5 m for NIGHT and DAY (100 formal samples).\n"
        "- Keep the same person, pose, camera mounting, and measured camera-to-person reference.\n"
        "- Near-field saturated bbox samples must be flagged, not silently discarded.\n"
        "- Candidate model should use bbox width + height / metric features and be validated cross-light:\n"
        "  train DAY -> test NIGHT, then train NIGHT -> test DAY.\n"
        "- Deploy new coefficients only when independent/cross-light MAE and RMSE improve.\n"
        "- Confidence is a detector score, not a distance variable; do not force it to be monotonic.\n"
    )

def export43(root,state):
    d=root/"4.3_LANE_SAFETY"; d.mkdir(exist_ok=True); summary=[]; items=[]
    for key,label in COND43:
        rr=conditions(state,"4.3").get(key,{}).get("rows",[]); items.append((label,rr))
        match=sum(1 for r in rr if r.get("status_match") is True)
        lg=[finite(r.get("left_gap_m")) for r in rr]; rg=[finite(r.get("right_gap_m")) for r in rr]; off=[finite(r.get("vehicle_offset_m")) for r in rr]
        lg=[x for x in lg if x is not None];rg=[x for x in rg if x is not None];off=[x for x in off if x is not None]
        center_err=[finite(r.get("centering_error_m")) for r in rr];center_err=[x for x in center_err if x is not None]
        status_err=[finite(r.get("status_error")) for r in rr];status_err=[x for x in status_err if x is not None]
        summary.append({"condition_id":key,"condition_name":label,"N":len(rr),"status_match_pct":100*match/len(rr) if rr else 0,
                        "status_error_rate_pct":100*statistics.fmean(status_err) if status_err else None,
                        "lane_offset_reference_m":0.0,
                        "lane_offset_actual_mean_m":statistics.fmean(off) if off else None,
                        "centering_bias_m":statistics.fmean(center_err) if center_err else None,
                        "centering_mae_m":statistics.fmean(abs(x) for x in center_err) if center_err else None,
                        "centering_rmse_m":math.sqrt(statistics.fmean(x*x for x in center_err)) if center_err else None,
                        "centering_p95_abs_error_m":percentile([abs(x) for x in center_err],.95),
                        "left_gap_mean_m":statistics.fmean(lg) if lg else None,"right_gap_mean_m":statistics.fmean(rg) if rg else None,
                        "offset_mean_m":statistics.fmean(off) if off else None})
    write_csv(d/"summary.csv",summary,list(summary[0].keys()))
    raw43=rows_all(state,"4.3")
    if raw43:
        fields43=[]
        for r in raw43:
            for k in r:
                if k not in fields43:fields43.append(k)
        write_csv(d/"raw_data.csv",raw43,fields43)
    write_xlsx(d/"raw_data.xlsx",[(key,conditions(state,"4.3").get(key,{}).get("rows",[])) for key,_ in COND43]+[("summary",summary)])
    labels=[x[1] for x in COND43]
    save_bar(d/"lane_status_graph.png","Kesesuaian Status Lane Safety",labels,[r["status_match_pct"] for r in summary],"Kesesuaian [%]")
    fig=plt.figure(figsize=(11,5));ax=fig.add_subplot(111)
    xidx=list(range(len(labels)))
    ax.plot(xidx,[0.0]*len(labels),linestyle="--",label="Reference center = 0 m")
    ax.plot(xidx,[finite(r.get("lane_offset_actual_mean_m")) or 0 for r in summary],marker="o",label="Actual mean")
    ax.set_xticks(xidx,labels,rotation=20);ax.set_ylabel("Offset [m]");ax.set_title("Reference vs Aktual Offset Kendaraan")
    ax.grid(True,alpha=.25);ax.legend();fig.tight_layout();fig.savefig(d/"offset_graph.png",dpi=160);plt.close(fig)
    fig=plt.figure(figsize=(11,5));ax=fig.add_subplot(111);x=list(range(len(labels)));w=.36
    ax.bar([i-w/2 for i in x],[finite(r["left_gap_mean_m"]) or 0 for r in summary],w,label="LEFT")
    ax.bar([i+w/2 for i in x],[finite(r["right_gap_mean_m"]) or 0 for r in summary],w,label="RIGHT")
    ax.set_xticks(x,labels,rotation=20);ax.set_ylabel("Gap [m]");ax.set_title("Perbandingan Gap Kiri/Kanan");ax.legend();ax.grid(True,axis="y",alpha=.25)
    fig.tight_layout();fig.savefig(d/"gap_comparison.png",dpi=160);plt.close(fig)
    montage(d/"composite_6_condition.png","Dokumentasi Enam Kondisi Lane Safety",items,root,3)
    write_csv(d/"graph_source_manifest.csv",[
        {"graph":"lane_status_graph.png","raw_source":"raw_data.csv","columns":"condition_id,left_status,right_status,status_match"},
        {"graph":"offset_graph.png","raw_source":"raw_data.csv","columns":"condition_id,lane_offset_reference_m,lane_offset_actual_m,centering_error_m,abs_centering_error_m"},
        {"graph":"gap_comparison.png","raw_source":"raw_data.csv","columns":"condition_id,left_gap_m,right_gap_m"},
        {"graph":"composite_6_condition.png","raw_source":"raw_data.csv + images/","columns":"condition_id,frame_filename"}],
        ["graph","raw_source","columns"])
def export44(root,state):
    d=root/"4.4_REALTIME_PERFORMANCE";d.mkdir(exist_ok=True);summary=[]
    metrics=["fps","inference_ms","processing_ms","frame_age_ms"]
    for key,label in COND44:
        rr=conditions(state,"4.4").get(key,{}).get("rows",[])
        row={"condition_id":key,"condition_name":label,"samples":len(rr)}
        for m in metrics:
            sm=stats([r.get(m) for r in rr])
            for k,v in sm.items():row[f"{m}_{k}"]=v
            row[f"{m}_p95"]=percentile([r.get(m) for r in rr],.95)
        fps_ref=[finite(r.get("fps_reference_hz")) for r in rr];fps_ref=[x for x in fps_ref if x is not None]
        fps_act=[finite(r.get("fps_actual_hz")) for r in rr];fps_act=[x for x in fps_act if x is not None]
        fps_err=[finite(r.get("fps_error_hz")) for r in rr];fps_err=[x for x in fps_err if x is not None]
        inf_ref=[finite(r.get("inference_time_reference_ms")) for r in rr];inf_ref=[x for x in inf_ref if x is not None]
        inf_act=[finite(r.get("inference_time_actual_ms")) for r in rr];inf_act=[x for x in inf_act if x is not None]
        inf_err=[finite(r.get("inference_time_error_ms")) for r in rr];inf_err=[x for x in inf_err if x is not None]
        row["fps_reference_hz"]=statistics.fmean(fps_ref) if fps_ref else None
        row["fps_actual_mean_hz"]=statistics.fmean(fps_act) if fps_act else None
        row["fps_bias_hz"]=statistics.fmean(fps_err) if fps_err else None
        row["fps_mae_hz"]=statistics.fmean(abs(x) for x in fps_err) if fps_err else None
        row["fps_rmse_hz"]=math.sqrt(statistics.fmean(x*x for x in fps_err)) if fps_err else None
        row["fps_p95_abs_error_hz"]=percentile([abs(x) for x in fps_err],.95)
        row["inference_time_reference_ms"]=statistics.fmean(inf_ref) if inf_ref else None
        row["inference_time_actual_mean_ms"]=statistics.fmean(inf_act) if inf_act else None
        row["inference_time_bias_ms"]=statistics.fmean(inf_err) if inf_err else None
        row["inference_time_mae_ms"]=statistics.fmean(abs(x) for x in inf_err) if inf_err else None
        row["inference_time_rmse_ms"]=math.sqrt(statistics.fmean(x*x for x in inf_err)) if inf_err else None
        row["inference_time_p95_abs_error_ms"]=percentile([abs(x) for x in inf_err],.95)
        row["stale_samples"]=sum(1 for r in rr if r.get("stream_status")!="LIVE")
        row["availability_pct"]=100.0*(len(rr)-row["stale_samples"])/len(rr) if rr else None
        row["dropped_frame_total"]=sum(int(r.get("dropped_frame") or 0) for r in rr)
        summary.append(row)
    fields=list(summary[0].keys());write_csv(d/"summary.csv",summary,fields)
    raw44=rows_all(state,"4.4")
    if raw44:
        fields44=[]
        for r in raw44:
            for k in r:
                if k not in fields44:fields44.append(k)
        write_csv(d/"raw_data.csv",raw44,fields44)
    write_xlsx(d/"raw_data.xlsx",[(key,conditions(state,"4.4").get(key,{}).get("rows",[])) for key,_ in COND44]+[("summary",summary)])
    labels=[x[1] for x in COND44]
    for metric,name in [("fps","fps_comparison.png"),("inference_ms","inference_comparison.png"),("frame_age_ms","frame_age_comparison.png")]:
        save_bar(d/name,metric.replace("_"," ").title(),labels,[finite(r.get(metric+"_mean")) or 0 for r in summary],metric)
    xidx=list(range(len(labels)))
    fig=plt.figure(figsize=(10,5));ax=fig.add_subplot(111)
    ax.plot(xidx,[finite(r.get("fps_reference_hz")) or 0 for r in summary],linestyle="--",marker="o",label="Reference / target")
    ax.plot(xidx,[finite(r.get("fps_actual_mean_hz")) or 0 for r in summary],marker="o",label="Actual mean")
    ax.set_xticks(xidx,labels,rotation=15);ax.set_ylabel("FPS");ax.set_title("Target vs Aktual Pipeline FPS");ax.grid(True,alpha=.25);ax.legend()
    fig.tight_layout();fig.savefig(d/"fps_target_vs_actual.png",dpi=160);plt.close(fig)
    fig=plt.figure(figsize=(10,5));ax=fig.add_subplot(111)
    ax.plot(xidx,[finite(r.get("inference_time_reference_ms")) or 0 for r in summary],linestyle="--",marker="o",label="Reference period")
    ax.plot(xidx,[finite(r.get("inference_time_actual_mean_ms")) or 0 for r in summary],marker="o",label="Actual inference")
    ax.set_xticks(xidx,labels,rotation=15);ax.set_ylabel("ms");ax.set_title("Reference Period vs Aktual Inference Time");ax.grid(True,alpha=.25);ax.legend()
    fig.tight_layout();fig.savefig(d/"inference_target_vs_actual.png",dpi=160);plt.close(fig)
    with (d/"performance_report.txt").open("w") as f:
        for r in summary:f.write(json.dumps(r,ensure_ascii=False)+"\n")
    write_csv(d/"graph_source_manifest.csv",[
        {"graph":"fps_comparison.png","raw_source":"raw_data.csv","columns":"condition,fps"},
        {"graph":"fps_target_vs_actual.png","raw_source":"raw_data.csv","columns":"condition,fps_reference_hz,fps_actual_hz,fps_error_hz,fps_performance_margin_hz"},
        {"graph":"inference_comparison.png","raw_source":"raw_data.csv","columns":"condition,inference_ms"},
        {"graph":"inference_target_vs_actual.png","raw_source":"raw_data.csv","columns":"condition,inference_time_reference_ms,inference_time_actual_ms,inference_time_error_ms,inference_time_margin_ms"},
        {"graph":"frame_age_comparison.png","raw_source":"raw_data.csv","columns":"condition,frame_age_ms"}],
        ["graph","raw_source","columns"])

def export45(root,state):
    d=root/"4.5_VEHICLE_RESPONSE";d.mkdir(exist_ok=True);rows=rows_all(state,"4.5")
    obs=d/"obstacle_response";lane=d/"lane_response";obs.mkdir(exist_ok=True);lane.mkdir(exist_ok=True)
    obs_keys={"obstacle_trial_1","obstacle_trial_2","obstacle_trial_3"}
    lane_keys={"lane_trial_1","lane_trial_2","lane_trial_3"}
    obsrows=[r for r in rows if str(r.get("condition","")) in obs_keys]
    lanerows=[r for r in rows if str(r.get("condition","")) in lane_keys]
    fields=["capture_id","timestamp","mode","condition","trial_index",
            "reference_type","reference_source","actual_source","reference_value","actual_value","signed_error","abs_error",
            "goal_distance_m","goal_reference_m","goal_actual_progress_m","goal_signed_error_m","goal_abs_error_m","goal_endpoint_error_m","final_map_x","final_map_y",
            "lane_offset_reference_m","lane_offset_actual_m","lane_offset_error_m","lane_offset_abs_error_m",
            "heading_reference_rad","heading_actual_rad","heading_signed_error_rad","heading_abs_error_rad","heading_signed_error_deg","heading_abs_error_deg",
            "steering_reference_rad","steering_actual_feedback_rad","steering_signed_error_rad","steering_abs_error_rad","steering_signed_error_deg","steering_abs_error_deg",
            "left_status","right_status","initial_lane_status","final_lane_status","expected_status","actual_status","status_match","status_error",
            "human_detected","human_distance","human_confidence","reference_distance_m","distance_error_m",
            "obstacle_memory_latched","obstacle_memory_released","obstacle_memory_latched_seen","obstacle_memory_released_seen",
            "obstacle_memory_base_forward_m","obstacle_memory_odom_x","obstacle_memory_odom_y",
            "perception_decision","expected_action","actual_action","lane_evidence_source","lane_corridor_evidence_mode",
            "lane_left_boundary_source","lane_right_boundary_source","lane_left_gap_px","lane_right_gap_px",
            "lane_left_active_samples","lane_right_active_samples","lane_left_drivable_edge_samples","lane_right_drivable_edge_samples",
            "lane_classical_fallback_active","lane_classical_left_samples","lane_classical_right_samples",
            "drive_target","drive_actual","steering_target","steering_actual",
            "expected_steering_direction","steering_response_ms","steering_direction_match","peak_steering_actual_rad","web_trial_authority_seen",
            "command_sent","response_time_ms","recovery_ms","travel_m","forward_progress_m","lateral_displacement_m",
            "target_distance_m","clear_target_m","initial_heading_rad","current_heading_rad","heading_error_rad","heading_error_deg","heading_restored","heading_restored_once","lane_phase","lane_nav2_handoff","lane_nav2_handoff_at_ms","lane_nav2_authority_seen","initial_map_x","initial_map_y","initial_map_heading_rad","avoid_direction","human_start_confidence","human_start_display_x_fraction",
            "command_v_mps","command_w_rad_s","heartbeat_count","heartbeat_last_ms","heartbeat_age_ms","heartbeat_transport","heartbeat_sequence",
            "trial_person_only_planning_filter","obstacle_motion_released","obstacle_motion_released_at_ms","obstacle_memory_latched_gate_at_ms",
            "planner_gate_pass","planner_gate_started_ms","planner_gate_passed_ms","planner_gate_elapsed_ms","planner_gate_reason",
            "planner_gate_planning_points","planner_gate_local_costmap_aligned","planner_gate_global_costmap_post_latch",
            "planner_gate_path_post_costmap","planner_gate_mppi_post_path","planner_gate_path_length_m","planner_gate_path_heading_variation_rad",
            "planner_hold_active","planner_hold_count","planner_hold_recovery_count","planner_hold_reason",
            "planner_path_age_ms","planner_path_count","planner_path_length_m","planner_path_heading_variation_rad","planner_path_quality_ok",
            "final_vehicle_state","trial_reason","integration_pass","result","frame_filename"]
    write_csv(obs/"raw.csv",obsrows,fields);write_csv(lane/"raw.csv",lanerows,fields)
    def summarize(rr):
        out=[]
        for key in sorted({r.get("condition") for r in rr}):
            a=[r for r in rr if r.get("condition")==key]
            dry=sum(1 for r in a if r.get("mode")=="DRY_RUN")
            armed=sum(1 for r in a if r.get("mode")=="ARMED_VEHICLE_TEST")
            complete=sum(1 for r in a if str(r.get("result","")).lower()=="complete")
            stopped=sum(1 for r in a if r.get("final_vehicle_state")=="STOPPED")
            match=sum(1 for r in a if r.get("status_match") is True)
            fp=[finite(r.get("forward_progress_m")) for r in a if finite(r.get("forward_progress_m")) is not None]
            lat=[finite(r.get("lateral_displacement_m")) for r in a if finite(r.get("lateral_displacement_m")) is not None]
            goal_ref=[finite(r.get("goal_reference_m")) for r in a];goal_ref=[x for x in goal_ref if x is not None]
            goal_actual=[finite(r.get("goal_actual_progress_m")) for r in a];goal_actual=[x for x in goal_actual if x is not None]
            goal_err=[finite(r.get("goal_signed_error_m")) for r in a];goal_err=[x for x in goal_err if x is not None]
            endpoint=[finite(r.get("goal_endpoint_error_m")) for r in a];endpoint=[x for x in endpoint if x is not None]
            lane_err=[finite(r.get("lane_offset_error_m")) for r in a];lane_err=[x for x in lane_err if x is not None]
            heading_err=[finite(r.get("heading_signed_error_deg")) for r in a];heading_err=[x for x in heading_err if x is not None]
            steer_err=[finite(r.get("steering_signed_error_deg")) for r in a];steer_err=[x for x in steer_err if x is not None]
            heartbeat_age=[finite(r.get("heartbeat_age_ms")) for r in a];heartbeat_age=[x for x in heartbeat_age if x is not None]
            heartbeat_count=[finite(r.get("heartbeat_count")) for r in a];heartbeat_count=[x for x in heartbeat_count if x is not None]
            gate_elapsed=[finite(r.get("planner_gate_elapsed_ms")) for r in a];gate_elapsed=[x for x in gate_elapsed if x is not None]
            hold_count=[finite(r.get("planner_hold_count")) for r in a];hold_count=[x for x in hold_count if x is not None]
            hold_recovery=[finite(r.get("planner_hold_recovery_count")) for r in a];hold_recovery=[x for x in hold_recovery if x is not None]
            out.append({"condition":key,"N":len(a),"dry_run_N":dry,"armed_N":armed,"complete_N":complete,"stopped_N":stopped,
                        "status_match_N":match,
                        "goal_reference_m":statistics.fmean(goal_ref) if goal_ref else None,
                        "goal_actual_progress_m_mean":statistics.fmean(goal_actual) if goal_actual else None,
                        "goal_bias_m":statistics.fmean(goal_err) if goal_err else None,
                        "goal_mae_m":statistics.fmean(abs(x) for x in goal_err) if goal_err else None,
                        "goal_rmse_m":math.sqrt(statistics.fmean(x*x for x in goal_err)) if goal_err else None,
                        "goal_endpoint_error_mean_m":statistics.fmean(endpoint) if endpoint else None,
                        "lane_offset_mae_m":statistics.fmean(abs(x) for x in lane_err) if lane_err else None,
                        "heading_mae_deg":statistics.fmean(abs(x) for x in heading_err) if heading_err else None,
                        "steering_mae_deg":statistics.fmean(abs(x) for x in steer_err) if steer_err else None,
                        "heartbeat_age_max_ms":max(heartbeat_age) if heartbeat_age else None,
                        "heartbeat_count_max":max(heartbeat_count) if heartbeat_count else None,
                        "planner_gate_pass_N":sum(1 for r in a if r.get("planner_gate_pass") is True),
                        "planner_gate_elapsed_mean_ms":statistics.fmean(gate_elapsed) if gate_elapsed else None,
                        "planner_hold_count_max":max(hold_count) if hold_count else None,
                        "planner_hold_recovery_count_max":max(hold_recovery) if hold_recovery else None,
                        "person_only_planning_filter_N":sum(1 for r in a if r.get("trial_person_only_planning_filter") is True),
                        "forward_progress_m_mean":statistics.fmean(fp) if fp else None,
                        "lateral_displacement_m_mean":statistics.fmean(lat) if lat else None,
                        "command_sent_any":any(bool(r.get("command_sent")) for r in a),
                        "expected_actions":" | ".join(sorted({str(r.get("expected_action")) for r in a}))})
        return out
    so,sl=summarize(obsrows),summarize(lanerows)
    if so:write_csv(obs/"summary.csv",so,list(so[0].keys()))
    if sl:write_csv(lane/"summary.csv",sl,list(sl[0].keys()))
    write_xlsx(d/"raw_data.xlsx",[("obstacle",obsrows),("lane",lanerows),("obstacle_summary",so),("lane_summary",sl)])
    if obsrows:
        save_bar(d/"obstacle_response_graph.png","Respons Obstacle",[str(r.get("condition","")) for r in obsrows],[finite(r.get("human_confidence")) or 0 for r in obsrows],"Human Confidence")
        obs_armed=[r for r in obsrows if r.get("mode")=="ARMED_VEHICLE_TEST"]
        if obs_armed:
            labels_obs=[str(r.get("condition","")) for r in obs_armed];xobs=list(range(len(obs_armed)))
            fig=plt.figure(figsize=(10,5));ax=fig.add_subplot(111)
            ax.plot(xobs,[finite(r.get("goal_reference_m")) or 0 for r in obs_armed],linestyle="--",marker="o",label="Goal reference")
            ax.plot(xobs,[finite(r.get("goal_actual_progress_m")) or 0 for r in obs_armed],marker="o",label="Actual progress")
            ax.set_xticks(xobs,labels_obs,rotation=15);ax.set_ylabel("m");ax.set_title("4.5A Goal Reference vs Aktual");ax.grid(True,alpha=.25);ax.legend()
            fig.tight_layout();fig.savefig(d/"obstacle_goal_reference_vs_actual.png",dpi=160);plt.close(fig)
            save_bar(d/"obstacle_endpoint_error.png","4.5A Endpoint Error",labels_obs,[finite(r.get("goal_endpoint_error_m")) or 0 for r in obs_armed],"Endpoint Error [m]")
    if lanerows:
        action_score={"STOP":0,"STRAIGHT_1M_THEN_STOP":1,"STRAIGHT_FORWARD_2M_THEN_STOP":1,
                      "YELLOW_WARNING_ANY_SIDE_NO_STEER_THEN_STOP":2,
                      "RED_ANY_SIDE_RECENTER_TO_GREEN_THEN_STOP":3,
                      "AUTO_SIDE_RED_RECENTER_GREEN_RESTORE_INITIAL_HEADING_FORWARD_5M_STOP":4,
                      "SLOW_RECENTER_RIGHT_UNTIL_GREEN_THEN_STRAIGHT_TO_FORWARD_2M_STOP":2,
                      "SLOW_RECENTER_LEFT_UNTIL_GREEN_THEN_STRAIGHT_TO_FORWARD_2M_STOP":2,
                      "RECENTER_RIGHT_UNTIL_GREEN_THEN_STRAIGHT_TO_FORWARD_2M_STOP":3,
                      "RECENTER_LEFT_UNTIL_GREEN_THEN_STRAIGHT_TO_FORWARD_2M_STOP":3}
        save_bar(d/"lane_response_graph.png","Respons Lane Safety",[str(r.get("condition","")) for r in lanerows],[action_score.get(str(r.get("expected_action","")),0) for r in lanerows],"Action Code")
        armed=[r for r in lanerows if r.get("mode")=="ARMED_VEHICLE_TEST"]
        if armed:
            save_bar(d/"forward_progress_graph.png","Forward Progress Lane ARMED",[str(r.get("condition","")) for r in armed],[finite(r.get("forward_progress_m")) or 0 for r in armed],"Forward Progress [m]")
            save_bar(d/"lateral_displacement_graph.png","Lateral Displacement Lane ARMED",[str(r.get("condition","")) for r in armed],[finite(r.get("lateral_displacement_m")) or 0 for r in armed],"Lateral Displacement [m]")
            labels_arm=[str(r.get("condition","")) for r in armed];xidx=list(range(len(armed)))
            fig=plt.figure(figsize=(10,5));ax=fig.add_subplot(111)
            ax.plot(xidx,[finite(r.get("goal_reference_m")) or 0 for r in armed],linestyle="--",marker="o",label="Goal reference")
            ax.plot(xidx,[finite(r.get("goal_actual_progress_m")) or 0 for r in armed],marker="o",label="Actual progress")
            ax.set_xticks(xidx,labels_arm,rotation=15);ax.set_ylabel("m");ax.set_title("4.5B Goal Reference vs Aktual");ax.grid(True,alpha=.25);ax.legend()
            fig.tight_layout();fig.savefig(d/"lane_goal_reference_vs_actual.png",dpi=160);plt.close(fig)
            save_bar(d/"lane_heading_error.png","4.5B Absolute Heading Error",labels_arm,[finite(r.get("heading_abs_error_deg")) or 0 for r in armed],"|Heading Error| [deg]")
            save_bar(d/"lane_steering_error.png","4.5B Absolute Steering Tracking Error",labels_arm,[finite(r.get("steering_abs_error_deg")) or 0 for r in armed],"|Steering Error| [deg]")
    (d/"response_report.txt").write_text("4.5 protocol: 4.5A uses three repeated obstacle trials with a 5 m Nav2 goal. 4.5B uses three repeated 5 m lane recenter trials: each starts from RED on either side, recovers to GREEN/GREEN, restores the vehicle heading to the heading captured at trial start, then hands motion to Nav2/MPPI to continue to the 5 m goal and finish in STOP. DRY RUN must pass before ARMED.\n")
    write_csv(d/"graph_source_manifest.csv",[
        {"graph":"obstacle_response_graph.png","raw_source":"obstacle_response/raw.csv","columns":"condition,human_confidence"},
        {"graph":"lane_response_graph.png","raw_source":"lane_response/raw.csv","columns":"condition,expected_action"},
        {"graph":"obstacle_goal_reference_vs_actual.png","raw_source":"obstacle_response/raw.csv","columns":"condition,mode,goal_reference_m,goal_actual_progress_m,goal_signed_error_m,goal_abs_error_m"},
        {"graph":"obstacle_endpoint_error.png","raw_source":"obstacle_response/raw.csv","columns":"condition,mode,goal_endpoint_error_m,goal_x_map,goal_y_map,final_map_x,final_map_y"},
        {"graph":"forward_progress_graph.png","raw_source":"lane_response/raw.csv","columns":"condition,mode,forward_progress_m"},
        {"graph":"lateral_displacement_graph.png","raw_source":"lane_response/raw.csv","columns":"condition,mode,lateral_displacement_m"},
        {"graph":"lane_goal_reference_vs_actual.png","raw_source":"lane_response/raw.csv","columns":"condition,mode,goal_reference_m,goal_actual_progress_m,goal_signed_error_m,goal_abs_error_m"},
        {"graph":"lane_heading_error.png","raw_source":"lane_response/raw.csv","columns":"condition,mode,heading_reference_rad,heading_actual_rad,heading_signed_error_deg,heading_abs_error_deg"},
        {"graph":"lane_steering_error.png","raw_source":"lane_response/raw.csv","columns":"condition,mode,steering_reference_rad,steering_actual_feedback_rad,steering_signed_error_deg,steering_abs_error_deg"}],
        ["graph","raw_source","columns"])

def manifest(root,state,metadata):
    mapping={"4.1":"4.1_VALIDASI_SISTEM","4.2":"4.2_HUMAN_DETECTION","4.3":"4.3_LANE_SAFETY","4.4":"4.4_REALTIME_PERFORMANCE","4.5":"4.5_VEHICLE_RESPONSE"}
    protocol45={"obstacle_trial_1","obstacle_trial_2","obstacle_trial_3",
                "lane_trial_1","lane_trial_2","lane_trial_3"}
    for key,folder in mapping.items():
        d=root/folder;rr=rows_all(state,key);conds=list(conditions(state,key).keys())
        if key=="4.5":
            conds=[c for c in conds if c in protocol45]
            rr=[r for r in rr if str(r.get("condition","")) in protocol45]
        m={"session_id":state.get("session_id"),"created_at":state.get("created_at"),"camera_flip":True,
           "coordinate_reference":"DISPLAY_OVERVIEW_MIRRORED","subchapter":key,"conditions":conds,"sample_count":len(rr),
           "config_snapshot":metadata.get("config_snapshot"),"software_version":state.get("software_version")}
        m["file_count"]=len([p for p in d.rglob("*") if p.is_file()])
        (d/"manifest.json").write_text(json.dumps(m,indent=2,ensure_ascii=False))
    allfiles=[str(p.relative_to(root)) for p in root.rglob("*") if p.is_file() and p.name!="manifest.json"]
    rootm={"session_id":state.get("session_id"),"created_at":state.get("created_at"),"camera_flip":True,
           "coordinate_reference":"DISPLAY_OVERVIEW_MIRRORED","conditions":{k:list(conditions(state,k).keys()) for k in mapping},
           "sample_count":sum(len(rows_all(state,k)) for k in mapping),"file_count":len(allfiles),
           "config_snapshot":metadata.get("config_snapshot"),"software_version":state.get("software_version"),"files":allfiles}
    (root/"manifest.json").write_text(json.dumps(rootm,indent=2,ensure_ascii=False))

def main():
    ap=argparse.ArgumentParser();ap.add_argument("--session",required=True);a=ap.parse_args()
    root=Path(a.session).resolve();state=load_json(root/"state.json");meta=load_json(root/"metadata.json")
    if not state or not root.name.startswith("BAB4_PERSEPSI_"):raise SystemExit("invalid session")
    export41(root,state);export42(root,state);export43(root,state);export44(root,state);export45(root,state);manifest(root,state,meta)
    print(json.dumps({"ok":True,"session":root.name,"files":len(list(root.rglob("*")))},ensure_ascii=False))
if __name__=="__main__":main()
