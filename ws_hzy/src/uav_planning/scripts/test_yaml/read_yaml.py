#! /usr/bin/env python
import yaml
import io

def get_yaml_data(fileDir):
    resList = []

    fo = io.open(fileDir,"r",encoding='utf-8')

    res = yaml.load(fo,Loader=yaml.FullLoader)
  #  print(res)

  #  del res[0]
  #  for one in res:
  #      resList.append((one["data"],one["resp"]))
    return res

#if __name__ == '__main__':
#    res=get_yaml_data('./data.yaml')
    #for one in res:
    #    print(one)
#    print (type(res['point1']['y']))

