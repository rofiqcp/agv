#include <QCoreApplication>
#include <QTextStream>
#include "src/navigation/gui/agv_experiment_catalog.hpp"
int main(int argc,char**argv){QCoreApplication app(argc,argv);QTextStream o(stdout);
for(const QString &sub:{"steering","navigation","perception"})for(const auto&x:buildExperimentCatalog(sub)){
 for(auto it=x.liveSeries.cbegin();it!=x.liveSeries.cend();++it)o<<sub<<"\t"<<x.id<<"\t"<<it.key()<<"\t"<<it.value()<<"\n";
}return 0;}
