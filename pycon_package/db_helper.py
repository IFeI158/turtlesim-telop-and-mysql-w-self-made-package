import pymysql

DB_CONFIG = dict(
    host = "172.22.160.1",
    user = "cho",
    password = "123",
    database = "rosdb",
    charset = "utf8"
)

class DB:
    def __init__(self, **config):
        self.config = config

    def connect(self):
        return pymysql.connect(**self.config)

    def db_pose(self, id, x, y, theta, time):
        sql = "INSERT INTO TURTLE_POSE (id, x, y, theta, time) VALUES (%s, %s, %s, %s, %s)"
        try:
            with self.connect() as conn:
                with conn.cursor() as cur:
                    cur.execute(sql, (id, x, y, theta, time))
                    conn.commit()
            return True
        except Exception as e:
            print("DB Error:", e)
            return False
