import numpy as np
from sets import Set


class MySet(object):

    def __init__(self):
        pass

    def map2Set(self, xSet, ySet):
        '''映射两个集合'''
        dict = {}
        return dict

    def intersection(self, xSet, ySet):
        '''求两个集合的交集'''
        pass

    def union(self, xSet, ySet):
        '''求两个集合的并集'''
        pass

    def difference(self, xSet, ySet):
        '''求两个集合的差集'''
        pass


class Apriori(object):
    ''' Apriori 布尔关联规则挖掘
    '''

    CkSet = {}

    def __init__(self, dataSet, minSupport):
        self.minSupport = minSupport
        self.dataSet = dataSet

    def createC1(self):
        ''' 构建集合C1，C1是大小为1的所有候选项集的集合。
            C1是空列表，用来存储所有不重复的项值。如果某个物品项没有在C1中出现，则将其添加到C1中。
            这里并不是简单地每个物品项，而是添加只包含该物品项的一个列表。Python不能创建只有一个整
            数的集合，因此这里实现必须使用列表
        '''
        C1 = []
        for transaction in self.dataSet:
            for item in transaction:
                s = [item]
                if not s in C1:
                    C1.append(s)
        C1.sort()

        # frozenset是指被“冰冻”的集合，就是说它们是不可改变
        return map(frozenset, C1)

    def associationRules(self):
        '''关联规则'''
        pass

    def supportFilter(self, supportDict):
        '''支持度筛选'''
        retSet = Set()
        for k in supportDict.keys():
            if supportDict[k] >= self.minSupport:
                retSet.add(k)
        return retSet

    def confidence(self, itemA, itemB):
        '''置信度'''
        if self.CkSet.has_key(itemA) and self.CkSet.has_key(itemB):
            itemUnion = list(Set(itemA) | Set(itemB))
            if self.CkSet.has_key(itemUnion):
                return self.CkSet[itemUnion] / self.CkSet[itemA]
        return 0

    def statistic(self, Ck):
        '''统计Ck集在样本空间中出现的次数'''
        dict = {}
        C = Set(Ck)
        transaction = map(frozenset, self.dataSet)
        for item in C:
            for trans in transaction:
                if len(trans) == len(Set(item) | Set(trans)):
                    if dict.has_key(item):
                        dict[item] += 1
                    else:
                        dict[item] = 1
        return dict

    def items(self):
        print self.CkSet

    def createCk(self, C, n, maxLen):
        '''n = k - 1 && n >= 1 && n < maxLen'''
        if n < 1 or n >= maxLen:
            return

        if n == 1:
            C1Dict = self.statistic(C)
            C1 = self.supportFilter(C1Dict)
            for item in C1:
                self.CkSet[item] = C1Dict[item]

        CkList = []
        for si in C:
            for ni in C:
                unionSet = Set(si) | Set(ni)
                if len(unionSet) - 1 == len(si):
                    if unionSet not in CkList:
                        CkList.append(list(unionSet))
        CkDict = self.statistic(map(frozenset, CkList))
        Ck = Set()
        for k in CkDict.keys():
            if CkDict[k] >= self.minSupport:
                Ck.add(k)
                self.CkSet[k] = CkDict[k]
        self.createCk(Ck, n + 1, maxLen)


samples = [
    ["I1", "I2", "I5"],
    ["I2", "I4"],
    ["I2", "I3"],
    ["I1", "I2", "I4"],
    ["I1", "I3"],
    ["I2", "I3"],
    ["I1", "I3"],
    ["I1", "I2", "I3", "I5"],
    ["I1", "I2", "I3"]
]


def loadDataSet():
    return samples

print '+++++++++++++++++++++++++++++++'
ap = Apriori(loadDataSet(), minSupport=2)
C1 = ap.createC1()
maxlen = 0
for item in samples:
    if len(item) > maxlen:
        maxlen = len(item)
ap.createCk(C1, 1, maxlen)
for k in ap.CkSet.keys():
    if len(k) == 1:
        print k, ":", ap.CkSet[k]
for k in ap.CkSet.keys():
    if len(k) == 2:
        print k, ":", ap.CkSet[k]
for k in ap.CkSet.keys():
    if len(k) == 3:
        print k, ":", ap.CkSet[k]
