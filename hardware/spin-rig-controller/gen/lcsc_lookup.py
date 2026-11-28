"""Finds an LCSC (JLC) part number for every MPN in design.py: the exact-MPN match with the most stock.
Writes lcsc.json and prints anything missing or short. Run with network access; results are cached."""
import json
import os
import sys
import time
import urllib.request

import design

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lcsc.json')


def search(kw, n=10):
    req = urllib.request.Request(
        'https://jlcpcb.com/api/overseas-pcb-order/v1/shoppingCart/smtGood/selectSmtComponentList',
        data=json.dumps({"keyword": kw, "currentPage": 1, "pageSize": n}).encode(),
        headers={'Content-Type': 'application/json', 'User-Agent': 'Mozilla/5.0'})
    d = json.load(urllib.request.urlopen(req, timeout=30))
    return (d.get('data') or {}).get('componentPageInfo', {}).get('list', []) or []


def norm(s):
    return ''.join(c for c in s.upper() if c.isalnum())


def main():
    have = json.load(open(OUT)) if os.path.exists(OUT) else {}
    need = {}
    for p in design.PARTS:
        if p.mpn and not p.dnp:
            need[p.mpn] = need.get(p.mpn, 0) + 1
    for mpn, qty in sorted(need.items()):
        if mpn in have and '--refresh' not in sys.argv:
            continue
        try:
            items = search(mpn)
        except Exception as e:
            print('ERR', mpn, e)
            continue
        exact = [it for it in items if norm(it.get('componentModelEn', '')).startswith(norm(mpn))]
        # JLC's Economic assembly can't take a part of product type 2 ("Standard Only"), so prefer any other;
        # then basic parts, then stock (C101: ROQANG's RVT1H101M0810 is type 2, Honor Elec's is not)
        exact.sort(key=lambda it: (it.get('componentProductType') != 2, it.get('componentLibraryType') == 'base',
                                   it.get('stockCount') or 0), reverse=True)
        if exact:
            it = exact[0]
            have[mpn] = it['componentCode']
            flag = '' if (it.get('stockCount') or 0) >= 5 * qty else '  LOW STOCK'
            if it.get('componentProductType') == 2:
                flag += '  STANDARD PCBA ONLY'
            print('%-26s %-10s %-26s stock=%-7s %s%s' % (mpn, it['componentCode'], it['componentModelEn'],
                                                        it.get('stockCount'), it.get('componentLibraryType'), flag))
        else:
            print('%-26s NOT FOUND  (%s)' % (mpn, ', '.join(it.get('componentModelEn', '') for it in items[:4])))
        time.sleep(0.3)
    json.dump(have, open(OUT, 'w'), indent=1, sort_keys=True)


if __name__ == '__main__':
    main()
