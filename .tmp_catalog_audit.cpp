#include <QCoreApplication>
#include <QTextStream>
#include <QSet>
#include "src/navigation/gui/agv_experiment_catalog.hpp"

int main(int argc,char**argv){QCoreApplication app(argc,argv);QTextStream o(stdout);
  for(const QString &sub:{"steering","navigation","perception"}){
    const auto v=buildExperimentCatalog(sub);QSet<QString> ids;int issues=0;
    o<<"DOMAIN\t"<<sub<<"\tLEAVES\t"<<v.size()<<"\n";
    for(const auto &x:v){QStringList err;
      if(ids.contains(x.id)) err<<"DUP_ID"; ids.insert(x.id);
      if(x.section.trimmed().isEmpty()) err<<"EMPTY_SECTION";
      if(x.tableNames.size()!=x.tableColumns.size()) err<<"TABLE_NAME_COUNT";
      if(!x.graphs.isEmpty() && x.graphs.size()!=x.graphCaptions.size()) err<<"GRAPH_SPEC_COUNT";
      if(x.tableColumns.isEmpty()&&x.graphCaptions.isEmpty()) err<<"NO_EVIDENCE";
      for(auto it=x.liveSeries.cbegin();it!=x.liveSeries.cend();++it) if(it.value().trimmed().isEmpty()) err<<"EMPTY_SERIES_PATH";
      QSet<QString> pp; for(const auto&p:x.parameterFields){
        if(p.yamlFileKey.isEmpty()!=p.yamlPath.isEmpty()) err<<"YAML_PAIR";
        const QString k=p.yamlFileKey+":"+p.yamlPath+":"+p.key; if(pp.contains(k))err<<"DUP_PARAM"; pp.insert(k);
      }
      for(const auto&g:x.graphs) if(g.type=="time_series") for(const auto&s:g.series) if(!x.liveSeries.contains(s)) err<<("GRAPH_SERIES_MISSING:"+s);
      if(!err.isEmpty()) issues++;
      o<<"LEAF\t"<<sub<<"\t"<<x.id<<"\t"<<x.groupId<<"\tT"<<x.tableColumns.size()<<"\tG"<<x.graphCaptions.size()<<"\tS"<<x.liveSeries.size()<<"\tP"<<x.parameterFields.size()<<"\t"<<err.join('|')<<"\t"<<x.section<<"\n";
    } o<<"ISSUES\t"<<sub<<"\t"<<issues<<"\n";
  } return 0; }
