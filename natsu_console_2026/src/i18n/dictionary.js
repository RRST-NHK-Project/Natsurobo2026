export const DICTIONARY = {
  // UI Labels
  timer: {
    ja: "競技時間",
    en: "Match Time"
  },
  phase: {
    ja: "フェーズ",
    en: "Phase"
  },
  
  // Phases
  phase1: {
    ja: "Phase 1: スタート・移動",
    en: "Phase 1: Start & Move"
  },
  phase2: {
    ja: "Phase 2: 大うなぎ捕獲",
    en: "Phase 2: Big Unagi"
  },
  phase3: {
    ja: "Phase 3: 対角移動・石倉登坂",
    en: "Phase 3: Diagonal & Climb"
  },
  phase4: {
    ja: "Phase 4: 小うなぎ投入",
    en: "Phase 4: Small Unagi Drop"
  },
  phase5: {
    ja: "Phase 5: Vゴール合図",
    en: "Phase 5: V-Goal Signal"
  },
  phase6: {
    ja: "Phase 6: バッファ",
    en: "Phase 6: Buffer"
  },
  
  // Basket Labels
  greenBasketA: {
    ja: "緑 A",
    en: "Green A"
  },
  greenBasketB: {
    ja: "緑 B",
    en: "Green B"
  },
  farthestBlueBasket: {
    ja: "最遠青（対角）",
    en: "Farthest Blue (Diagonal)"
  },
  nearbyBlueBasketA: {
    ja: "近接青 A",
    en: "Nearby Blue A"
  },
  nearbyBlueBasketB: {
    ja: "近接青 B",
    en: "Nearby Blue B"
  },
  otherBlueBasketC: {
    ja: "その他青 C",
    en: "Other Blue C"
  },
  otherBlueBasketD: {
    ja: "その他青 D",
    en: "Other Blue D"
  },
  
  // V-Goal Checker
  bigUnagiCaptured: {
    ja: "大うなぎ捕獲",
    en: "Big Unagi Captured"
  },
  occupiedBasketsCount: {
    ja: "占有カゴ数",
    en: "Occupied Baskets"
  },
  isOnIshikura: {
    ja: "石倉上に登坂",
    en: "On Ishikura"
  },
  readyToVGoal: {
    ja: "大漁合図可能",
    en: "READY TO V-GOAL"
  },
  fallbackMode: {
    ja: "フォールバック (Fallback)",
    en: "Fallback Mode"
  },
  vgoalTime: {
    ja: "Vゴール目標時刻",
    en: "V-Goal Target Time"
  },
  targetDropCount: {
    ja: "目標投入数",
    en: "Target Drop Count"
  },
  
  // Ammo & Score
  ammoRemaining: {
    ja: "小うなぎ残弾",
    en: "Small Unagi Ammo"
  },
  estimatedScore: {
    ja: "推定スコア",
    en: "Estimated Score"
  },
  score: {
    ja: "スコア",
    en: "Score"
  },
  points: {
    ja: "点",
    en: "pt"
  },
  
  // Buttons
  add: {
    ja: "+",
    en: "+"
  },
  subtract: {
    ja: "-",
    en: "-"
  },
  selectTarget: {
    ja: "ターゲット選択",
    en: "Select Target"
  },
  activate: {
    ja: "有効",
    en: "ON"
  },
  deactivate: {
    ja: "無効",
    en: "OFF"
  },
  startsAt: {
    ja: "開始",
    en: "Starts at"
  },
  endsAt: {
    ja: "終了",
    en: "Ends at"
  },
  
  // Warnings
  targetMarker: {
    ja: "★基準ターゲット",
    en: "★ Primary Target"
  },
  currentTarget: {
    ja: "現在のターゲット",
    en: "Current Target"
  }
};

export const getLocalizedText = (key, lang = "ja") => {
  if (DICTIONARY[key] && DICTIONARY[key][lang]) {
    return DICTIONARY[key][lang];
  }
  return key;
};
