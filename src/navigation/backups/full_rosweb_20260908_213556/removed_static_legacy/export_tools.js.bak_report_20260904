// Excel/CSV export helpers for the tuning workbench. Pure browser-side; no ROS/build dependency.
function expXml(v){return String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&apos;'}[c]))}
function expColName(n){let s='';for(n++;n>0;n=Math.floor((n-1)/26))s=String.fromCharCode(65+(n-1)%26)+s;return s}
function expCell(ref,v,style=3){const s=String(v??'');if(s!==''&&Number.isFinite(Number(s))&&/^[-+]?\d+(?:\.\d+)?(?:e[-+]?\d+)?$/i.test(s))return `<c r="${ref}" s="${style}"><v>${Number(s)}</v></c>`;return `<c r="${ref}" s="${style}" t="inlineStr"><is><t xml:space="preserve">${expXml(s)}</t></is></c>`}
function expCat(parts){const n=parts.reduce((a,p)=>a+p.length,0),out=new Uint8Array(n);let o=0;for(const p of parts){out.set(p,o);o+=p.length}return out}
function expLe16(n){return Uint8Array.of(n&255,(n>>>8)&255)}
function expLe32(n){return Uint8Array.of(n&255,(n>>>8)&255,(n>>>16)&255,(n>>>24)&255)}
function expCrc32(data){let c=0xffffffff;for(const b of data){c^=b;for(let k=0;k<8;k++)c=(c>>>1)^((c&1)?0xedb88320:0)}return(c^0xffffffff)>>>0}
function expZipStore(files){
  const enc=new TextEncoder(),locals=[],central=[];let offset=0;
  const now=new Date(),dt=((now.getHours()<<11)|(now.getMinutes()<<5)|(now.getSeconds()>>1))&0xffff;
  const dd=(((Math.max(1980,now.getFullYear())-1980)<<9)|((now.getMonth()+1)<<5)|now.getDate())&0xffff;
  for(const f of files){const name=enc.encode(f.name),data=typeof f.data==='string'?enc.encode(f.data):f.data,crc=expCrc32(data);
    const local=expCat([expLe32(0x04034b50),expLe16(20),expLe16(0x800),expLe16(0),expLe16(dt),expLe16(dd),expLe32(crc),expLe32(data.length),expLe32(data.length),expLe16(name.length),expLe16(0),name,data]);
    locals.push(local);central.push(expCat([expLe32(0x02014b50),expLe16(20),expLe16(20),expLe16(0x800),expLe16(0),expLe16(dt),expLe16(dd),expLe32(crc),expLe32(data.length),expLe32(data.length),expLe16(name.length),expLe16(0),expLe16(0),expLe16(0),expLe16(0),expLe32(0),expLe32(offset),name]));offset+=local.length}
  const cdir=expCat(central),end=expCat([expLe32(0x06054b50),expLe16(0),expLe16(0),expLe16(files.length),expLe16(files.length),expLe32(cdir.length),expLe32(offset),expLe16(0)]);
  return expCat([...locals,cdir,end])
}
function expTableMatrix(tableIndex){
  const cols=selectedExperiment?.tableColumns?.[tableIndex]||[],rows=tableRunRows.get(tableIndex)||[],timed=tableHasTimeAxis();
  return {head:timed?['t [s]',...cols]:cols,rows:rows.map(r=>timed?[r.sec,...r.values]:r.values)}
}
function expSheetXml(tableIndex){
  const {head,rows}=expTableMatrix(tableIndex),title=`${displayExperimentId(selectedExperiment)} ${displayExperimentTitle(selectedExperiment)}`;
  const tableName=selectedExperiment?.tableNames?.[tableIndex]||`Tabel ${tableIndex+1}`,lastCol=expColName(Math.max(0,head.length-1));
  const widths=head.map((h,ci)=>Math.min(60,Math.max(10,String(h).length+2,...rows.map(r=>String(r[ci]??'').length+2))));
  let xml=`<?xml version="1.0" encoding="UTF-8" standalone="yes"?><worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"><sheetViews><sheetView workbookViewId="0"><pane ySplit="5" topLeftCell="A6" activePane="bottomLeft" state="frozen"/></sheetView></sheetViews>`;
  xml+=`<cols>${widths.map((w,i)=>`<col min="${i+1}" max="${i+1}" width="${w}" customWidth="1"/>`).join('')}</cols><sheetData>`;
  xml+=`<row r="1" ht="24" customHeight="1">${expCell('A1',title,1)}</row><row r="2" ht="22" customHeight="1">${expCell('A2',tableName,1)}</row>`;
  xml+=`<row r="3">${expCell('A3',`Variation: ${document.getElementById('runVariation')?.value||''} | Condition: ${document.getElementById('runCondition')?.value||''} | 1 row/detik`,4)}</row><row r="4"></row>`;
  xml+=`<row r="5">${head.map((h,i)=>expCell(`${expColName(i)}5`,h,2)).join('')}</row>`;
  rows.forEach((r,ri)=>{const rn=ri+6;xml+=`<row r="${rn}">${r.map((v,ci)=>expCell(`${expColName(ci)}${rn}`,v,3)).join('')}</row>`});
  xml+=`</sheetData><autoFilter ref="A5:${lastCol}${Math.max(5,rows.length+5)}"/><mergeCells count="3"><mergeCell ref="A1:${lastCol}1"/><mergeCell ref="A2:${lastCol}2"/><mergeCell ref="A3:${lastCol}3"/></mergeCells>`;
  return xml+`<pageMargins left="0.25" right="0.25" top="0.5" bottom="0.5" header="0.2" footer="0.2"/></worksheet>`
}
function expStylesXml(){return `<?xml version="1.0" encoding="UTF-8" standalone="yes"?><styleSheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"><fonts count="3"><font><sz val="11"/><name val="Calibri"/></font><font><b/><color rgb="FFFFFFFF"/><sz val="11"/><name val="Calibri"/></font><font><b/><color rgb="FF173F35"/><sz val="15"/><name val="Calibri"/></font></fonts><fills count="4"><fill><patternFill patternType="none"/></fill><fill><patternFill patternType="gray125"/></fill><fill><patternFill patternType="solid"><fgColor rgb="FF1F4E78"/><bgColor indexed="64"/></patternFill></fill><fill><patternFill patternType="solid"><fgColor rgb="FFD9EAD3"/><bgColor indexed="64"/></patternFill></fill></fills><borders count="2"><border/><border><left style="thin"><color rgb="FFD0D7D5"/></left><right style="thin"><color rgb="FFD0D7D5"/></right><top style="thin"><color rgb="FFD0D7D5"/></top><bottom style="thin"><color rgb="FFD0D7D5"/></bottom></border></borders><cellStyleXfs count="1"><xf numFmtId="0" fontId="0" fillId="0" borderId="0"/></cellStyleXfs><cellXfs count="5"><xf numFmtId="0" fontId="0" fillId="0" borderId="0" xfId="0"/><xf numFmtId="0" fontId="2" fillId="3" borderId="0" xfId="0" applyFont="1" applyFill="1"/><xf numFmtId="0" fontId="1" fillId="2" borderId="1" xfId="0" applyFont="1" applyFill="1" applyBorder="1"><alignment horizontal="center" vertical="center" wrapText="1"/></xf><xf numFmtId="0" fontId="0" fillId="0" borderId="1" xfId="0" applyBorder="1"><alignment vertical="top" wrapText="1"/></xf><xf numFmtId="0" fontId="0" fillId="0" borderId="0" xfId="0"><alignment vertical="center"/></xf></cellXfs></styleSheet>`}
function saveExperimentExcel(){
  if(!selectedExperiment)return;
  const tables=(selectedExperiment.tableColumns||[]).map((_,i)=>i).filter(i=>(tableRunRows.get(i)||[]).length);
  if(!tables.length)return toast('Belum ada data hasil START/STOP untuk dibuat Excel',true);
  const sheets=tables.map((ti,i)=>({ti,name:`Tabel ${ti+1}`,rid:`rId${i+1}`})),files=[];
  files.push({name:'[Content_Types].xml',data:`<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="xml" ContentType="application/xml"/><Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/><Override PartName="/xl/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml"/>${sheets.map((s,i)=>`<Override PartName="/xl/worksheets/sheet${i+1}.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>`).join('')}</Types>`});
  files.push({name:'_rels/.rels',data:`<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/></Relationships>`});
  files.push({name:'xl/workbook.xml',data:`<?xml version="1.0" encoding="UTF-8" standalone="yes"?><workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"><sheets>${sheets.map((s,i)=>`<sheet name="${expXml(s.name)}" sheetId="${i+1}" r:id="${s.rid}"/>`).join('')}</sheets></workbook>`});
  files.push({name:'xl/_rels/workbook.xml.rels',data:`<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">${sheets.map((s,i)=>`<Relationship Id="${s.rid}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet${i+1}.xml"/>`).join('')}<Relationship Id="rIdStyles" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/></Relationships>`});
  files.push({name:'xl/styles.xml',data:expStylesXml()});
  sheets.forEach((s,i)=>files.push({name:`xl/worksheets/sheet${i+1}.xml`,data:expSheetXml(s.ti)}));
  const binary=expZipStore(files),safe=reportRunId();
  downloadBlob(`${safe}_${stamp()}.xlsx`,'application/vnd.openxmlformats-officedocument.spreadsheetml.sheet',binary);
  toast(`Excel ${sheets.length} sheet didownload`)
}
function expGuiCsv(tableIndex){const {head,rows}=expTableMatrix(tableIndex);if(!head.length||!rows.length)return null;const esc=v=>'"'+String(v??'').replaceAll('"','""')+'"';return [head.map(esc).join(','),...rows.map(r=>r.map(esc).join(','))].join('\n')+'\n'}
function saveAllGuiCsv(){
  if(!selectedExperiment)return;
  let saved=0;const safe=reportRunId(),tag=stamp();
  for(let i=0;i<(selectedExperiment.tableColumns||[]).length;i++){const csv=expGuiCsv(i);if(!csv)continue;downloadBlob(`${safe}_T${i+1}_${tag}.csv`,'text/csv;charset=utf-8',csv);saved++}
  toast(saved?`${saved} CSV sesuai tabel GUI didownload`:'Belum ada data hasil START/STOP',!saved)
}
window.addEventListener('DOMContentLoaded',()=>{
  const csv=document.getElementById('saveTableCsvBtn'),xlsx=document.getElementById('saveTableExcelBtn');
  if(csv)csv.onclick=saveAllGuiCsv;
  if(xlsx)xlsx.onclick=saveExperimentExcel;
});
